#!/usr/bin/env bash
# WHOLE-BODY (QP-WBC) dynamic TROT e2e for the Unitree Go2 (M14, Example B). The SAME robot, Gazebo
# world, and diagonal TrotGait as the model-free kinematic trot (e2e_go2_trot_kinematic.sh), but
# control_law: wbc — an inverse-dynamics QP (contact constraints, friction cones, torque limits,
# full floating-base dynamics), CLOSED-LOOP on the base. Two modes, via env vars:
#   BASE=ecm       (default)  base from Gazebo GROUND TRUTH — validates the WBC trot open-loop-in-base
#   BASE=estimate  EST=inekf  base from the InEKF /base_odom — the closed-loop sim2real trot
# Judged on the TRUE base (/base_truth_odom, never seen by control in estimate mode): forward +
# upright. ecm mode ASSERTS (the gate); estimate mode is REPORT-ONLY (a probe — closed-loop on the
# estimate is the known frontier, worse than the crawl; see make-the-go2-trot-whole-body.md).
#
# Usage: e2e_go2_trot_wbc.sh              # ground truth
#        BASE=estimate EST=inekf e2e_go2_trot_wbc.sh   # closed-loop on the InEKF estimate
set -o pipefail
BASE="${BASE:-ecm}"; EST="${EST:-inekf}"
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
dur, out = float(sys.argv[1]), sys.argv[2]
def rpy(o):
    x,y,z,w=o.x,o.y,o.z,o.w
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))*180/math.pi,
            math.asin(max(-1,min(1,2*(w*y-z*x))))*180/math.pi)
class S(Node):
    def __init__(s):
        super().__init__('e2etrotwbc'); s.rows=[]; s.est={}; s.t0=time.time()
        s.create_subscription(Odometry,'/base_truth_odom',s.ct,20)
        s.create_subscription(Odometry,'/base_odom',s.ce,20)
    def ct(s,m):
        p=m.pose.pose.position; r,pi=rpy(m.pose.pose.orientation)
        s.rows.append((time.time()-s.t0, p.x, p.y, p.z, r, pi))
    def ce(s,m):
        p=m.pose.pose.position; s.est[round(time.time()-s.t0,2)]=(p.x,p.y,p.z)
rclpy.init(); n=S(); t0=time.time()
while time.time()-t0<dur: rclpy.spin_once(n,timeout_sec=0.03)
with open(out,'a') as f:
    if len(n.rows)<100:
        f.write("FAIL n=%d\n"%len(n.rows)); raise SystemExit
    T=n.rows[-1][0]; win=6.0
    early=[r for r in n.rows if r[0]<win]; late=[r for r in n.rows if r[0]>T-win]
    mx_e=sum(r[1] for r in early)/len(early); mx_l=sum(r[1] for r in late)/len(late)
    zs=[r[3] for r in n.rows]; ys=[r[2] for r in n.rows]
    roll=[abs(r[4]) for r in n.rows]; pit=[abs(r[5]) for r in n.rows]
    # Estimate-vs-truth position error (only meaningful when the estimator is running).
    err=0.0
    if n.est:
        for r in n.rows:
            e=n.est.get(round(r[0],2))
            if e: err=max(err, math.hypot(r[1]-e[0], r[2]-e[1]))
    f.write("fwd=%.4f zmin=%.4f zmax=%.4f ysway=%.4f zbob=%.4f rollmax=%.2f pitchmax=%.2f "
            "esterr=%.4f n=%d\n"%(mx_l-mx_e, min(zs), max(zs), max(ys)-min(ys), max(zs)-min(zs),
                                  max(roll), max(pit), err, len(n.rows)))
PY

echo "BASE=$BASE EST=$EST"
# Ground truth (ecm) needs no estimator — launch light (like the kinematic e2e) so bring-up is
# ready by the detach. base_source:=estimate auto-enables the estimator regardless of this flag.
EST_ON=true; [ "$BASE" = "ecm" ] && EST_ON=false
timeout 56 ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_trot_wbc.yaml base_source:="$BASE" estimator:="$EST_ON" estimator_type:="$EST" \
  >/dev/null 2>&1 &
sleep 16
# Release the startup weld — send twice (the weld plugin may not be ready on the first shot).
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
sleep 2
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
sleep 4            # let the robot settle; the gait starts stepping (start_delay 6)
python3 "$CAP" 30 "$OUT"    # ~30 trot strides (period 1.0 s)

cat "$OUT"
BASE="$BASE" python3 - "$OUT" <<'PY'
import os, sys
line=open(sys.argv[1]).read().strip()
if "FAIL" in line or not line: print("FAIL: incomplete data"); sys.exit(1)
v={k:float(x) for k,x in (kv.split("=") for kv in line.split() if "=" in kv)}
estimate = os.environ.get("BASE") == "estimate"
walked  = v["fwd"] > 0.03                                  # net forward progress (m)
upright = 0.20 < v["zmin"] and v["zmax"] < 0.34 and v["rollmax"] < 12 and v["pitchmax"] < 12
tracks  = v["esterr"] < 0.10                               # est within 10 cm of truth (tight)
bounded = v["esterr"] < 0.30                               # est stays bounded (not diverging)
if estimate:
    # Closed-loop ON THE ESTIMATE. Once the M15 startup settle-gate removed the startup-timing
    # race (the real cause of M14's tip), the Go2 trots UPRIGHT + FORWARD on its own InEKF
    # estimate, reliably. So this mode now ASSERTS forward + upright + a BOUNDED estimate (not
    # diverging). The estimate's absolute position still drifts ~0.19 m from truth over 30 s
    # (tight-tracking < 0.10 m is not yet met) — a bounded, non-tipping estimator refinement,
    # reported but not gated (see make-the-go2-trot-whole-body.md).
    ok = walked and upright and bounded
    print("%s [estimate]: Go2 WBC trot on the InEKF estimate — forward=%.3f m (%s), "
          "upright(z=%.2f..%.2f roll=%.1f pitch=%.1f)=%s, est_err=%.3f m (bounded=%s, "
          "tight-track<0.10=%s) [y_sway=%.3f]" % (
        "PASS" if ok else "FAIL", v["fwd"], walked, v["zmin"], v["zmax"], v["rollmax"],
        v["pitchmax"], upright, v["esterr"], bounded, tracks, v["ysway"]))
    sys.exit(0 if ok else 1)
# Ground truth (ecm): the asserted gate — the WBC trots forward and upright, closed-loop in base.
ok = walked and upright
print("%s: Go2 WBC trot [ecm] — forward=%.3f m (%s), upright(z=%.2f..%.2f roll=%.1f pitch=%.1f)=%s "
      "[info: y_sway=%.3f z_bob=%.3f]" % (
    "PASS" if ok else "FAIL", v["fwd"], walked, v["zmin"], v["zmax"], v["rollmax"], v["pitchmax"],
    upright, v["ysway"], v["zbob"]))
sys.exit(0 if ok else 1)
PY
