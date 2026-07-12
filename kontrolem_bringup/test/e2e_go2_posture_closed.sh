#!/usr/bin/env bash
# Commanded postures ON THE ESTIMATE (M9 Step 4 — the M8 closed-loop stress test under
# INTENTIONAL motion, not just a push). Launched with base_source:=estimate, the WBC moves
# the Go2's body through the canned squat/sway/tilt/yaw cycle using ONLY the estimator's
# /base_odom — no ground truth in the control loop. We judge the TRUE base
# (/base_truth_odom, never seen by control): it must execute the postures AND stay stable,
# while the estimate keeps tracking the true base throughout the motion.
#
# Usage: e2e_go2_posture_closed.sh
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
            math.asin(max(-1,min(1,2*(w*y-z*x))))*180/math.pi,
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))*180/math.pi)
def qa(a,b):
    d=abs(sum(x*y for x,y in zip(a,b))); return 2*math.acos(max(-1,min(1,d)))*180/math.pi
class S(Node):
    def __init__(s):
        super().__init__('clpose'); s.e=None; s.t=None; s.rows=[]; s.perr=[]; s.oerr=[]
        s.create_subscription(Odometry,'/base_odom',s.ce,20)
        s.create_subscription(Odometry,'/base_truth_odom',s.ct,20)
    def ce(s,m): s.e=m
    def ct(s,m):
        s.t=m; p=m.pose.pose.position; r,pi,ya=rpy(m.pose.pose.orientation)
        s.rows.append((p.z,r,pi,ya))
        if s.e:
            pe=s.e.pose.pose.position
            s.perr.append(math.sqrt((pe.x-p.x)**2+(pe.y-p.y)**2+(pe.z-p.z)**2))
            qe=s.e.pose.pose.orientation; qt=m.pose.pose.orientation
            s.oerr.append(qa((qe.x,qe.y,qe.z,qe.w),(qt.x,qt.y,qt.z,qt.w)))
rclpy.init(); n=S(); t0=time.time()
while time.time()-t0<dur: rclpy.spin_once(n,timeout_sec=0.03)
with open(out,'a') as f:
    if len(n.rows)<50 or len(n.perr)<50:
        f.write("FAIL n=%d/%d\n"%(len(n.rows),len(n.perr))); raise SystemExit
    zs=[r[0] for r in n.rows]; rr=[r[1] for r in n.rows]; pp=[r[2] for r in n.rows]; yy=[r[3] for r in n.rows]
    f.write("z_min=%.4f z_max=%.4f roll_span=%.2f pitch_span=%.2f yaw_span=%.2f "
            "est_p_max=%.4f est_o_max=%.2f\n"%(
        min(zs),max(zs),max(rr)-min(rr),max(pp)-min(pp),max(yy)-min(yy),
        max(n.perr),max(n.oerr)))
PY

timeout 40 ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_posture_controllers.yaml base_source:=estimate >/dev/null 2>&1 &
sleep 16
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
python3 "$CAP" 16 "$OUT"

cat "$OUT"
python3 - "$OUT" <<'PY'
import sys
line=open(sys.argv[1]).read().strip()
if "FAIL" in line or not line:
    print("FAIL: incomplete data"); sys.exit(1)
v={k:float(x) for k,x in (kv.split("=") for kv in line.split() if "=" in kv)}
# TRUE base executed the postures (all four motions) and stayed stable, while control saw
# only the estimate; the estimate tracked the true base throughout the commanded motion.
moved = ((v["z_max"]-v["z_min"]) > 0.015 and v["roll_span"] > 4.0 and v["pitch_span"] > 3.0 and
         v["yaw_span"] > 5.0)
stable = 0.24 < v["z_min"] and v["z_max"] < 0.34
tracked = v["est_p_max"] < 0.01 and v["est_o_max"] < 1.0
ok = moved and stable and tracked
print("%s: Go2 postures ON ESTIMATE — moved(zspan=%.3f roll=%.1f pitch=%.1f yaw=%.1f)=%s "
      "stable=%s estimate_tracks(pos=%.4f ori=%.2f)=%s" % (
    "PASS" if ok else "FAIL", v["z_max"]-v["z_min"], v["roll_span"], v["pitch_span"],
    v["yaw_span"], moved, stable, v["est_p_max"], v["est_o_max"], tracked))
sys.exit(0 if ok else 1)
PY
