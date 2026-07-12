#!/usr/bin/env bash
# Commanded-posture e2e for the Unitree Go2 (M9). The SAME WBC that STANDS the Go2 now
# MOVES its body through a range of postures while the feet stay planted, driven by the
# Tracking dialect. Two phases, both headless, on Gazebo ground truth:
#   CANNED: reference_type base_pose runs a squat/sway/tilt/yaw cycle; assert the true base
#           executes all four motions (z oscillates, roll/pitch/yaw sweep) and stays stable.
#   LIVE:   reference_type live; publish a ~/base_target Twist and assert the base moves to
#           the commanded pose, then returns to nominal on a zero command.
# Judged on the OdometryPublisher ground truth (/base_truth_odom).
#
# Usage: e2e_go2_posture.sh
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
tag, dur, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
def rpy(o):
    x,y,z,w=o.x,o.y,o.z,o.w
    r=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p=math.asin(max(-1,min(1,2*(w*y-z*x))))
    ya=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return r*180/math.pi,p*180/math.pi,ya*180/math.pi
class S(Node):
    def __init__(s):
        super().__init__('e2epose'); s.rows=[]
        s.create_subscription(Odometry,'/base_truth_odom',s.cb,20)
    def cb(s,m):
        p=m.pose.pose.position; r,pi,ya=rpy(m.pose.pose.orientation)
        s.rows.append((p.z,r,pi,ya))
rclpy.init(); n=S(); t0=time.time()
while time.time()-t0<dur: rclpy.spin_once(n,timeout_sec=0.03)
with open(out,'a') as f:
    if len(n.rows)<50:
        f.write("%s FAIL n=%d\n"%(tag,len(n.rows))); raise SystemExit
    zs=[r[0] for r in n.rows]; rr=[r[1] for r in n.rows]
    pp=[r[2] for r in n.rows]; yy=[r[3] for r in n.rows]
    f.write("%s z_min=%.4f z_max=%.4f z_final=%.4f roll_span=%.2f pitch_span=%.2f "
            "yaw_span=%.2f roll_max=%.2f pitch_max=%.2f\n"%(
        tag,min(zs),max(zs),zs[-1],max(rr)-min(rr),max(pp)-min(pp),max(yy)-min(yy),
        max(abs(v) for v in rr),max(abs(v) for v in pp)))
PY

# ---- Phase 1: CANNED squat/sway/tilt/yaw cycle -----------------------------------------
timeout 40 ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_posture_controllers.yaml >/dev/null 2>&1 &
sleep 16
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
python3 "$CAP" CANNED 16 "$OUT"
cleanup_phase() { pkill -9 -f "ign gazebo"; pkill -9 -f ruby; pkill -9 -f "ros2 launch"; \
  pkill -9 -f robot_state_publisher; pkill -9 -f parameter_bridge; pkill -9 -f spawner; }
cleanup_phase; sleep 3

# ---- Phase 2: LIVE ~/base_target command -----------------------------------------------
export ROS_DOMAIN_ID=$((ROS_DOMAIN_ID + 1))
timeout 40 ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_posture_live_controllers.yaml >/dev/null 2>&1 &
sleep 16
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
sleep 2
# Command: squat down 1.5 cm + pitch nose-down ~5 deg, hold, then measure.
ros2 topic pub --once /kontrolem_controller/base_target geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: -0.015}, angular: {x: 0.0, y: 0.09, z: 0.0}}" >/dev/null 2>&1
sleep 3
python3 "$CAP" LIVE_CMD 3 "$OUT"
# Return to nominal.
ros2 topic pub --once /kontrolem_controller/base_target geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
sleep 3
python3 "$CAP" LIVE_HOME 3 "$OUT"

cat "$OUT"
python3 - "$OUT" <<'PY'
import sys
L={l.split()[0]:l for l in open(sys.argv[1]) if l.strip()}
if any("FAIL" in L.get(k,"F") for k in ("CANNED","LIVE_CMD","LIVE_HOME")) or \
   not all(k in L for k in ("CANNED","LIVE_CMD","LIVE_HOME")):
    print("FAIL: incomplete data"); sys.exit(1)
def v(t): return {k:float(x) for k,x in (kv.split("=") for kv in L[t].split()[1:] if "=" in kv)}
c,cmd,home=v("CANNED"),v("LIVE_CMD"),v("LIVE_HOME")
# CANNED: all four motions executed, base stayed stable/upright.
canned = ((c["z_max"]-c["z_min"]) > 0.015 and c["roll_span"] > 4.0 and c["pitch_span"] > 3.0 and
          c["yaw_span"] > 5.0 and 0.24 < c["z_min"] and c["z_max"] < 0.34)
# LIVE: commanded pose reached (squatted below nominal 0.2868, pitched); home returns near nominal.
live_cmd = cmd["z_final"] < 0.283 and cmd["pitch_max"] > 3.0
live_home = home["z_final"] > 0.284 and home["pitch_max"] < 2.0
ok = canned and live_cmd and live_home
print("%s: Go2 postures — canned(zspan=%.3f roll=%.1f pitch=%.1f yaw=%.1f)=%s "
      "live_cmd(z=%.3f pitch=%.1f)=%s live_home(z=%.3f pitch=%.1f)=%s" % (
    "PASS" if ok else "FAIL", c["z_max"]-c["z_min"], c["roll_span"], c["pitch_span"],
    c["yaw_span"], canned, cmd["z_final"], cmd["pitch_max"], live_cmd,
    home["z_final"], home["pitch_max"], live_home))
sys.exit(0 if ok else 1)
PY
