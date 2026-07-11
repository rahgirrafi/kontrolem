#!/usr/bin/env bash
# CLOSED-LOOP validation (M8, Step 4): the Go2 stands and rejects a push using ONLY its
# own estimate — no ground truth in the control loop. Launched with base_source:=estimate,
# GzBaseStateSystem feeds the WBC the BaseEstimatorController's /base_odom instead of the
# ECM. We judge success on the TRUE base (the OdometryPublisher ground truth
# /base_truth_odom, which the controller never sees): it must hold stance and recover
# from the shove. We also log the estimate-vs-truth error throughout to confirm the loop
# stays honest. This is the definitive sim-to-real proof — control closes on the estimate.
#
# Usage: e2e_go2_estimator_closed.sh [push_N] [run_s]
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PUSH="${1:-5000.0}"       # lateral shove (N, one 2 ms step); the true base must recover
RUN="${2:-60}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 90 + 100))}"
OUT="$(mktemp)"; CAP="$(mktemp)"

cleanup() {
  pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f ruby 2>/dev/null
  pkill -9 -f robot_state_publisher 2>/dev/null; pkill -9 -f "ros2 launch" 2>/dev/null
  pkill -9 -f parameter_bridge 2>/dev/null; pkill -9 -f spawner 2>/dev/null
  rm -f "$OUT" "$CAP"
}
trap cleanup EXIT

cat > "$CAP" <<'PY'
import math, sys, time, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
tag, dur, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]

def quat_angle(a, b):
    d = abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3])
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))

class S(Node):
    def __init__(s):
        super().__init__('e2e_closed')
        s.est = None; s.tru = None
        s.ty = []; s.tz = []; s.perr = []; s.oerr = []
        s.create_subscription(Odometry, '/base_odom', s.cb_est, 20)
        s.create_subscription(Odometry, '/base_truth_odom', s.cb_tru, 20)
    def cb_est(s, m): s.est = m; s.pair()
    def cb_tru(s, m):
        s.tru = m
        s.ty.append(m.pose.pose.position.y)   # TRUE lateral (the real proof)
        s.tz.append(m.pose.pose.position.z)   # TRUE height
        s.pair()
    def pair(s):
        if s.est is None or s.tru is None: return
        pe = s.est.pose.pose.position; pt = s.tru.pose.pose.position
        s.perr.append(math.sqrt((pe.x-pt.x)**2 + (pe.y-pt.y)**2 + (pe.z-pt.z)**2))
        qe = s.est.pose.pose.orientation; qt = s.tru.pose.pose.orientation
        s.oerr.append(quat_angle((qe.x,qe.y,qe.z,qe.w), (qt.x,qt.y,qt.z,qt.w)))

rclpy.init(); n = S(); t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
with open(out, 'a') as f:
    if len(n.tz) < 50 or len(n.perr) < 50:
        f.write("%s FAIL n=%d/%d\n" % (tag, len(n.tz), len(n.perr)))
    else:
        f.write("%s ty_peak=%.4f ty_final=%.4f tz_min=%.4f tz_final=%.4f "
                "est_p_max=%.4f est_o_max_deg=%.3f\n" % (
            tag, max(n.ty, key=abs), n.ty[-1], min(n.tz), n.tz[-1],
            max(n.perr), max(n.oerr) * 180.0 / math.pi))
PY

timeout "$RUN" ros2 launch kontrolem_bringup go2_gz.launch.py base_source:=estimate >/dev/null 2>&1 &
sleep 16   # gazebo + spawn + weld + WBC/estimator activation + IMU + /base_odom flowing

# Hand the WBC (running on the ESTIMATE) the robot; let it settle.
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
sleep 3
python3 "$CAP" STANCE 3 "$OUT"

# Shove the base; the TRUE base must recover though control sees only the estimate.
ign topic -t /world/go2/wrench -m ignition.msgs.EntityWrench \
  -p "entity {name:\"go2::base\" type:LINK} wrench {force {y: $PUSH}}" >/dev/null 2>&1
python3 "$CAP" PUSH 10 "$OUT"

cat "$OUT"
python3 - "$OUT" <<'PY'
import sys
lines = {l.split()[0]: l for l in open(sys.argv[1]) if l.strip()}
if "STANCE" not in lines or "PUSH" not in lines or "FAIL" in lines.get("STANCE","F")+lines.get("PUSH","F"):
    print("FAIL: incomplete data — closed loop may not have come up"); sys.exit(1)
def vals(tag): return {k: float(v) for k, v in (kv.split("=") for kv in lines[tag].split()[1:] if "=" in kv)}
s, p = vals("STANCE"), vals("PUSH")
# TRUE base stands on the estimate: height in band, laterally settled, and the estimate
# tracks truth (bounded). Then the TRUE base is visibly shoved and RECOVERS — all with
# the control loop seeing only the estimate.
stance = (0.25 <= s["tz_min"] and s["tz_final"] <= 0.33 and abs(s["ty_final"]) < 0.03 and
          s["est_p_max"] < 0.03)
pushed = 0.02 <= abs(p["ty_peak"]) <= 0.20
recovered = abs(p["ty_final"]) < 0.03 and 0.25 <= p["tz_final"] <= 0.33 and p["est_p_max"] < 0.05
ok = stance and pushed and recovered
print("%s: CLOSED-LOOP Go2 on estimate — stance(true_z=%.3f est_err=%.3f)=%s "
      "push(true_y_peak=%.3f)=%s recovery(true_y=%.3f true_z=%.3f est_err=%.3f)=%s" % (
    "PASS" if ok else "FAIL", s["tz_final"], s["est_p_max"], stance,
    p["ty_peak"], pushed, p["ty_final"], p["tz_final"], p["est_p_max"], recovered))
sys.exit(0 if ok else 1)
PY
