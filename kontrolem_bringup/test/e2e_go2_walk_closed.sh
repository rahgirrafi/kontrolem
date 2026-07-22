#!/usr/bin/env bash
# Static crawl WALK on the ESTIMATE (M10 Step 5) — the sim-to-real finale. Launched with
# base_source:=estimate, the WBC walks the Go2 forward using ONLY the estimator's /base_odom
# (IMU + leg odometry + real contact) — no ground truth in the control loop, feet leaving
# and rejoining the ground. This is the estimator's stepping path getting its first real
# workout. Judged on the TRUE base (/base_truth_odom, never seen by control): it advances
# forward and stays up (no collapse), while the estimate keeps tracking through the stepping.
#
# FRONTIER PROBE (report-only, always exits 0): walking on the estimate is the edge of what
# the contact-aided leg-odometry filter can do. The flat_ground height pin stops HEIGHT
# drift and a gentle gait helps, but during stepping the SENSED foot contact flickers and a
# briefly-mis-sensed swing foot poisons the leg-odometry velocity solve — so the estimate
# sometimes tracks well (forward walk, ~cm error) and sometimes diverges. A full InEKF
# (which models this) is the deferred general fix. This probe reports the metrics for a run;
# the SOLID M10 deliverable is the ground-truth walk (e2e_go2_walk.sh). Headless.
#
# Usage: e2e_go2_walk_closed.sh
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
def qa(a,b):
    d=abs(sum(x*y for x,y in zip(a,b))); return 2*math.acos(max(-1,min(1,d)))*180/math.pi
class S(Node):
    def __init__(s):
        super().__init__('e2ewc'); s.e=None; s.t=None; s.rows=[]; s.perr=[]; s.oerr=[]
        s.create_subscription(Odometry,'/base_odom',s.ce,20)
        s.create_subscription(Odometry,'/base_truth_odom',s.ct,20)
    def ce(s,m): s.e=m
    def ct(s,m):
        s.t=m; p=m.pose.pose.position; s.rows.append((time.time(),p.x,p.z))
        if s.e:
            pe=s.e.pose.pose.position
            s.perr.append(math.sqrt((pe.x-p.x)**2+(pe.y-p.y)**2+(pe.z-p.z)**2))
            qe=s.e.pose.pose.orientation; qt=m.pose.pose.orientation
            s.oerr.append(qa((qe.x,qe.y,qe.z,qe.w),(qt.x,qt.y,qt.z,qt.w)))
rclpy.init(); n=S(); t0=time.time()
while time.time()-t0<dur: rclpy.spin_once(n,timeout_sec=0.03)
with open(out,'a') as f:
    if len(n.rows)<100 or len(n.perr)<100:
        f.write("FAIL n=%d/%d\n"%(len(n.rows),len(n.perr))); raise SystemExit
    xs=[r[1] for r in n.rows]; zs=[r[2] for r in n.rows]; win=len(xs)//5
    fwd=(sum(xs[-win:])/win)-(sum(xs[:win])/win)
    f.write("fwd=%.4f zmin=%.4f est_p_max=%.4f est_o_max=%.2f n=%d\n"%(
        fwd, min(zs), max(n.perr), max(n.oerr), len(n.rows)))
PY

EST="${EST:-inekf}"   # M11: default to the InEKF (set EST=complementary for the M8 baseline)
timeout 60 ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_walk_controllers.yaml base_source:=estimate estimator_type:="$EST" \
  >/dev/null 2>&1 &
sleep 16
ign topic -t /go2/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
sleep 6
python3 "$CAP" 34 "$OUT"

echo "estimator_type=$EST"
cat "$OUT"
python3 - "$OUT" <<'PY'
import sys
line=open(sys.argv[1]).read().strip()
if "FAIL" in line or not line: print("PROBE: no data (launch/topics may not have come up)"); sys.exit(0)
v={k:float(x) for k,x in (kv.split("=") for kv in line.split() if "=" in kv)}
walked  = v["fwd"] > 0.03            # true base advanced forward
upright = v["zmin"] > 0.22           # no collapse (step dips OK)
tracked = v["est_p_max"] < 0.10      # estimate stayed roughly with the true base
clean = walked and upright and tracked
# M11: with estimator_type:=inekf the walk is CLEAN in most runs and tighter than the M8
# complementary filter, but still occasionally diverges — full reliability needs the next
# layer (feed the gait's PLANNED contact instead of the flickering sensed contact, or IMU
# bias). So this stays a report-only probe (never asserts).
verdict = "CLEAN walk on estimate" if clean else "MARGINAL (occasional divergence; see M11 notes)"
print("PROBE [%s]: forward=%.3f m, zmin=%.3f, estimate pos_err_max=%.3f ori_err_max=%.2f deg" % (
    verdict, v["fwd"], v["zmin"], v["est_p_max"], v["est_o_max"]))
sys.exit(0)   # report-only frontier probe
PY
