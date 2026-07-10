#!/usr/bin/env bash
# End-to-end BASE-STATE round-trip (M6.3). A free body is dropped (tilted) in
# Gazebo; its ground-truth odometry is bridged to ROS and fed through the
# OdometryBaseBridge ros2_control SENSOR -> the 13 floating_base state interfaces ->
# FloatingStateProbe, which reassembles a manifold State (BaseStateSensor). This
# exercises the whole non-joint-state path against a NON-custom producer (no custom
# sim, no gz_ros2_control). Two independent checks:
#   (chain)  /floating_state_probe/base_state shows real motion + a unit quaternion,
#            proving Gazebo base state reached the controller as a valid manifold State.
#   (frame)  SE(3) forward-integration of the /base_odom stream (body-frame twist,
#            odom's own timestamps) predicts the next pose to <1 cm — proving Gazebo's
#            twist is body-frame, matching Pinocchio's free-flyer (else it diverges).
# Headless; standalone; hard timeout.
#
# Usage: e2e_floating_box_gz.sh [run_s]
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
RUN="${1:-45}"
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
import sys, time, math, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
dur = float(sys.argv[1]); out = sys.argv[2]
state = []   # (pos_z, quat_norm, frame_resid)
odom = []    # (t, px,py,pz, qx,qy,qz,qw, vx,vy,vz, wx,wy,wz)

class S(Node):
    def __init__(s):
        super().__init__('e2e_m63')
        s.create_subscription(Float64MultiArray, '/floating_state_probe/base_state', s.cs, 10)
        s.create_subscription(Odometry, '/base_odom', s.co, 10)
    def cs(s, m):
        d = m.data
        if len(d) >= 15:
            state.append((d[2], d[13], d[14]))
    def co(s, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position; o = m.pose.pose.orientation
        l = m.twist.twist.linear; a = m.twist.twist.angular
        odom.append((t, p.x, p.y, p.z, o.x, o.y, o.z, o.w, l.x, l.y, l.z, a.x, a.y, a.z))

def rot(q, v):  # rotate vector v by quaternion q=(x,y,z,w)
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q_vec, v); v' = v + w*t + cross(q_vec, t)
    tx = 2 * (y * vz - z * vy); ty = 2 * (z * vx - x * vz); tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))

rclpy.init(); n = S(); t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)

with open(out, 'w') as f:
    if len(state) < 100 or len(odom) < 100:
        f.write("FAIL state_n=%d odom_n=%d\n" % (len(state), len(odom))); sys.exit()
    zs = [r[0] for r in state]
    zrange = max(zs) - min(zs)
    min_qnorm = min(r[1] for r in state)
    max_resid = max(r[2] for r in state)
    # Frame check: SE(3) forward-integrate consecutive odom samples (body-frame twist).
    errs = []
    prev = None
    for s in odom:
        if prev is not None:
            dt = s[0] - prev[0]
            if 1e-4 < dt < 0.05:
                pos_prev = prev[1:4]; q_prev = prev[4:8]; vlin = prev[8:11]
                wv = rot(q_prev, vlin)  # body-frame linear vel -> world
                pred = (pos_prev[0] + wv[0] * dt, pos_prev[1] + wv[1] * dt, pos_prev[2] + wv[2] * dt)
                act = s[1:4]
                errs.append(math.sqrt(sum((pred[i] - act[i]) ** 2 for i in range(3))))
        prev = s
    if not errs:
        f.write("FAIL no_odom_pairs\n"); sys.exit()
    mean_err = sum(errs) / len(errs); max_err = max(errs)
    f.write("state_n=%d odom_n=%d zrange=%.3f min_qnorm=%.6f max_resid=%.4f "
            "frame_mean=%.5f frame_max=%.5f\n" % (
        len(state), len(odom), zrange, min_qnorm, max_resid, mean_err, max_err))
PY

timeout "$RUN" ros2 launch kontrolem_bringup floating_box_gz.launch.py >/dev/null 2>&1 &
sleep 10   # Gazebo startup + bridge + probe activation (probe timer is 6 s)

python3 "$CAP" 12 "$OUT"

read -r LINE < "$OUT"
echo "$LINE"
python3 - "$LINE" <<'PY'
import sys
line = sys.argv[1]
if line.startswith("FAIL") or "frame_mean=" not in line:
    print("FAIL: incomplete data — Gazebo/bridge/probe may not have come up (%s)" % line.strip())
    sys.exit(1)
d = dict(kv.split("=") for kv in line.split() if "=" in kv)
zrange = float(d["zrange"]); min_qnorm = float(d["min_qnorm"])
frame_mean = float(d["frame_mean"]); frame_max = float(d["frame_max"])
moved = zrange > 0.5              # real Gazebo motion reached the controller
unit = min_qnorm > 0.999         # BaseStateSensor produced a valid manifold quaternion
body_frame = frame_mean < 0.01   # odom twist is body-frame (matches Pinocchio free-flyer)
ok = moved and unit and body_frame
print("%s: Gazebo base-state — moved(zrange=%.2fm)=%s unit_quat(min=%.5f)=%s "
      "body_frame(mean_err=%.4f max=%.4f)=%s" % (
    "PASS" if ok else "FAIL", zrange, moved, min_qnorm, unit, frame_mean, frame_max, body_frame))
sys.exit(0 if ok else 1)
PY
