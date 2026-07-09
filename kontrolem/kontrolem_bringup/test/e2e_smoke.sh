#!/usr/bin/env bash
# End-to-end smoke test: launch a Kontrol'Em demo, watch a joint settle, PASS/FAIL.
#
# Deliberately a standalone script (hard timeouts, self-cleaning) rather than a
# colcon launch-test: full ros2_control launches are flaky under some CI/harness
# setups, and a hanging/flaky test in the suite is worse than a script you run on
# purpose. Run it after building + sourcing the workspace.
#
# Usage: e2e_smoke.sh <launch_file> <joint> <threshold> [settle_s] [run_s] [target]
#   e.g. e2e_smoke.sh cart_pole.launch.py        pole_joint  0.05
#        e2e_smoke.sh cart_double_pole.launch.py pole1_joint 0.05
#        e2e_smoke.sh quad_stand.launch.py       knee_FL     0.2  9 14 -1.4
# PASS when |joint - target| < threshold after the settle window (target default 0:
# a regulator drives to 0; a WBC holds a non-zero standing posture).
set -o pipefail

LAUNCH="${1:?launch file}"; JOINT="${2:?joint}"; THRESH="${3:?threshold}"
SETTLE="${4:-9.0}"; RUN="${5:-14}"; TARGET="${6:-0.0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 90 + 100))}"
TS="$(mktemp)"; LOG="$(mktemp)"

cleanup() { pkill -9 -f ros2_control_node 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; pkill -9 -f spawner 2>/dev/null; rm -f "$TS" "$LOG"; }
trap cleanup EXIT

timeout "$RUN" ros2 launch kontrolem_bringup "$LAUNCH" > "$LOG" 2>&1 &

# Probe /joint_states; record the joint value after the settle window.
timeout "$RUN" python3 - "$JOINT" "$SETTLE" > "$TS" 2>/dev/null <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
joint, settle = sys.argv[1], float(sys.argv[2])
t0 = time.time()
class S(Node):
    def __init__(s):
        super().__init__('e2e'); s.create_subscription(JointState, '/joint_states', s.cb, 10)
    def cb(s, m):
        d = dict(zip(m.name, m.position))
        if joint in d and time.time() - t0 > settle:
            print(d[joint]); sys.stdout.flush()
rclpy.init(); n = S()
try: rclpy.spin(n)
except Exception: pass
PY

VAL="$(tail -1 "$TS")"
if [ -z "$VAL" ]; then
  echo "FAIL: no post-settle reading for '$JOINT' (launch may not have come up)"; exit 1
fi
# abs(VAL - TARGET) < THRESH ?
python3 - "$VAL" "$THRESH" "$JOINT" "$TARGET" <<'PY'
import sys
val, thr, joint, tgt = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3], float(sys.argv[4])
ok = abs(val - tgt) < thr
print(f"{'PASS' if ok else 'FAIL'}: {joint} settled to {val:+.4f} (|. - {tgt:+.3f}| < {thr})")
sys.exit(0 if ok else 1)
PY
