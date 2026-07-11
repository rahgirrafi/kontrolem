#!/usr/bin/env bash
# Open-loop validation of the floating-base state estimator (M8, Step 3). The
# BaseEstimatorController runs ALONGSIDE the WBC and publishes its estimate on
# /base_odom, while the WBC still stands the Go2 on Gazebo GROUND TRUTH
# (GzBaseStateSystem) — so this measures the estimator's accuracy WITHOUT yet trusting
# it in the control loop (the gate before closing the loop in Step 4). We compare the
# estimate /base_odom against the OdometryPublisher ground truth /base_truth_odom
# through a stance settle and a lateral push, and assert the position/orientation error
# stays small.
#
# This is an ESTIMATOR-ACCURACY test, not a control-stress test: it uses a fair
# perturbation (a real ~0.9 g lateral shove) and asserts tight tracking throughout. The
# extreme 5000 N slam lives in e2e_go2_gz.sh (WBC robustness) and, definitively, in the
# closed-loop Step 4 test — the estimate need only be good enough to control on, which a
# violent-transient orientation peak (the complementary filter's bandwidth limit, an
# InEKF upgrade target) does not preclude.
#
# Usage: e2e_go2_estimator.sh [push_N] [run_s]
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PUSH="${1:-1000.0}"
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
    # geodesic angle between two quaternions (x,y,z,w)
    d = abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3])
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))

class S(Node):
    def __init__(s):
        super().__init__('e2e_est')
        s.est = None; s.tru = None
        s.perr = []; s.zerr = []; s.oerr = []
        s.create_subscription(Odometry, '/base_odom', s.cb_est, 20)
        s.create_subscription(Odometry, '/base_truth_odom', s.cb_tru, 20)
    def cb_est(s, m): s.est = m; s.pair()
    def cb_tru(s, m): s.tru = m; s.pair()
    def pair(s):
        if s.est is None or s.tru is None:
            return
        pe = s.est.pose.pose.position; pt = s.tru.pose.pose.position
        s.perr.append(math.sqrt((pe.x-pt.x)**2 + (pe.y-pt.y)**2 + (pe.z-pt.z)**2))
        s.zerr.append(abs(pe.z - pt.z))          # standing HEIGHT error (what balance needs)
        qe = s.est.pose.pose.orientation; qt = s.tru.pose.pose.orientation
        s.oerr.append(quat_angle((qe.x,qe.y,qe.z,qe.w), (qt.x,qt.y,qt.z,qt.w)))

rclpy.init(); n = S(); t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
with open(out, 'a') as f:
    if len(n.perr) < 50:
        f.write("%s FAIL n=%d\n" % (tag, len(n.perr)))
    else:
        f.write("%s n=%d p_max=%.4f z_max=%.4f z_final=%.4f o_max_deg=%.3f o_final_deg=%.3f\n" % (
            tag, len(n.perr), max(n.perr), max(n.zerr), n.zerr[-1],
            max(n.oerr) * 180.0 / math.pi, n.oerr[-1] * 180.0 / math.pi))
PY

timeout "$RUN" ros2 launch kontrolem_bringup go2_gz.launch.py >/dev/null 2>&1 &
sleep 16   # gazebo + spawn + weld + WBC/estimator activation + IMU flowing

# Hand the WBC the robot; let the estimate settle against the moving base.
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
sleep 3
python3 "$CAP" STANCE 4 "$OUT"

# Shove the base laterally; the estimate must keep tracking through the transient.
ign topic -t /world/go2/wrench -m ignition.msgs.EntityWrench \
  -p "entity {name:\"go2::base\" type:LINK} wrench {force {y: $PUSH}}" >/dev/null 2>&1
python3 "$CAP" PUSH 10 "$OUT"

cat "$OUT"
python3 - "$OUT" <<'PY'
import sys
lines = {l.split()[0]: l for l in open(sys.argv[1]) if l.strip()}
if "STANCE" not in lines or "PUSH" not in lines or "FAIL" in lines.get("STANCE","F")+lines.get("PUSH","F"):
    print("FAIL: incomplete data — estimator/ground-truth topics may not have come up"); sys.exit(1)
def vals(tag): return {k: float(v) for k, v in (kv.split("=") for kv in lines[tag].split()[1:] if "=" in kv)}
s, p = vals("STANCE"), vals("PUSH")
# Gate on what standing balance needs: HEIGHT and ORIENTATION, tight. In STANCE the
# height error < 1 cm and orientation < 0.5 deg; after the PUSH both RECOVER (height
# < 1.5 cm, orientation < 0.5 deg) and the orientation PEAK stays bounded < 2.5 deg.
# Full-3D position (p_max) is reported but NOT gated tightly: leg odometry has no
# absolute horizontal reference, so a few cm of xy drift through the violent detach-drop
# is expected (an InEKF / absolute-aiding upgrade target) and does not preclude stable
# standing — the offset is ~constant, and closing the loop in Step 4 is the definitive
# proof the estimate is good enough to control on.
stance_ok = s["z_max"] < 0.01 and s["o_max_deg"] < 0.5 and s["p_max"] < 0.02
push_ok = (p["z_final"] < 0.015 and p["o_final_deg"] < 0.5 and p["o_max_deg"] < 2.5 and
           p["p_max"] < 0.02)
ok = stance_ok and push_ok
print("%s: estimator open-loop — stance(z_max=%.4f o_max=%.3f p_max=%.4f)=%s "
      "push(z_final=%.4f o_peak=%.3f o_final=%.3f p_max=%.4f)=%s" % (
    "PASS" if ok else "FAIL", s["z_max"], s["o_max_deg"], s["p_max"], stance_ok,
    p["z_final"], p["o_max_deg"], p["o_final_deg"], p["p_max"], push_ok))
sys.exit(0 if ok else 1)
PY
