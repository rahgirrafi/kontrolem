#!/usr/bin/env bash
# Kinematic (model-FREE) TROT e2e for the Unitree Go2 (M13, Example A). The SAME robot + Gazebo
# world as the WBC demos, but control_law: kinematic_gait — no dynamics, no QP, no estimator. The
# KinematicGaitController samples a diagonal TrotGait, IKs each foot to its target, and holds the
# legs with a joint PD. Judged on the OdometryPublisher ground truth (/base_truth_odom): the base
# advances forward and stays upright. Unlike the crawl there is NO lateral weight-shift (a trot
# does not sway), so we do NOT assert y-sway; forward + upright is the proof it trotted. Headless.
#
# Usage: e2e_go2_trot_kinematic.sh
set -o pipefail
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
        super().__init__('e2etrot'); s.rows=[]; s.t0=time.time()
        s.create_subscription(Odometry,'/base_truth_odom',s.cb,20)
    def cb(s,m):
        p=m.pose.pose.position; r,pi=rpy(m.pose.pose.orientation)
        s.rows.append((time.time()-s.t0, p.x, p.y, p.z, r, pi))
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
    f.write("fwd=%.4f zmin=%.4f zmax=%.4f ysway=%.4f zbob=%.4f rollmax=%.2f pitchmax=%.2f n=%d\n"%(
        mx_l-mx_e, min(zs), max(zs), max(ys)-min(ys), max(zs)-min(zs), max(roll), max(pit), len(n.rows)))
PY

timeout 56 ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_trot_kinematic.yaml estimator:=false base_source:=ecm >/dev/null 2>&1 &
sleep 16
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
sleep 6            # let the robot settle; the gait starts stepping (start_delay)
python3 "$CAP" 30 "$OUT"    # ~37 trot strides (period 0.8 s)

cat "$OUT"
python3 - "$OUT" <<'PY'
import sys
line=open(sys.argv[1]).read().strip()
if "FAIL" in line or not line: print("FAIL: incomplete data"); sys.exit(1)
v={k:float(x) for k,x in (kv.split("=") for kv in line.split() if "=" in kv)}
walked  = v["fwd"] > 0.03                                  # net forward progress (m)
upright = 0.20 < v["zmin"] and v["zmax"] < 0.34 and v["rollmax"] < 12 and v["pitchmax"] < 12
ok = walked and upright
print("%s: Go2 kinematic trot — forward=%.3f m (%s), upright(z=%.2f..%.2f roll=%.1f pitch=%.1f)=%s "
      "[info: y_sway=%.3f z_bob=%.3f]" % (
    "PASS" if ok else "FAIL", v["fwd"], walked, v["zmin"], v["zmax"], v["rollmax"],
    v["pitchmax"], upright, v["ysway"], v["zbob"]))
sys.exit(0 if ok else 1)
PY
