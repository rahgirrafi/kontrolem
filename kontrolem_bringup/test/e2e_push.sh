#!/usr/bin/env bash
# End-to-end PUSH-RECOVERY test: launch a WBC standing demo whose sim delivers a
# scheduled external base push, and verify the base (a) DEVIATES during the push
# (proving the disturbance landed) and (b) RETURNS to nominal afterward (proving
# the WBC recovered), with the controller feasible on every tick.
#
# Reads the base position from the controller's ControllerDiagnostics (q[0], q[1]
# are the base x, y). Standalone script with a hard timeout, like e2e_smoke.sh.
#
# Usage: e2e_push.sh [launch] [min_disp_m] [max_final_m] [run_s]
set -o pipefail

LAUNCH="${1:-quad_push.launch.py}"
MIN_DISP="${2:-0.010}"   # base must move at least this far (m) — the push must land
MAX_FINAL="${3:-0.006}"  # base must return within this (m) — recovery
RUN="${4:-20}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 90 + 100))}"
OUT="$(mktemp)"

cleanup() { pkill -9 -f ros2_control_node 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; pkill -9 -f spawner 2>/dev/null; rm -f "$OUT"; }
trap cleanup EXIT

timeout "$RUN" ros2 launch kontrolem_bringup "$LAUNCH" >/dev/null 2>&1 &

timeout "$RUN" python3 - "$MIN_DISP" "$MAX_FINAL" > "$OUT" 2>/dev/null <<'PY'
import sys, math, rclpy
from rclpy.node import Node
from kontrolem_msgs.msg import ControllerDiagnostics
min_disp, max_final = float(sys.argv[1]), float(sys.argv[2])
rows = []
class S(Node):
    def __init__(s):
        super().__init__('e2epush')
        s.create_subscription(ControllerDiagnostics, '/kontrolem_controller/diagnostics', s.cb, 10)
    def cb(s, m):
        if len(m.q) >= 3:
            rows.append((m.q[0], m.q[1], bool(m.ok)))
rclpy.init(); n = S()
try: rclpy.spin(n)
except Exception: pass
if len(rows) < 50:
    print("FAIL: no/low diagnostics (%d samples) — launch may not have come up" % len(rows))
    sys.exit(1)
maxh = max(math.hypot(x, y) for x, y, _ in rows)          # peak deviation
tail = rows[-25:]
fx = sum(x for x, y, _ in tail) / len(tail)
fy = sum(y for x, y, _ in tail) / len(tail)
finh = math.hypot(fx, fy)                                  # settled deviation
ok_all = all(o for _, _, o in rows)
ok = (maxh >= min_disp) and (finh <= max_final) and ok_all
print("%s: pushed %.1f mm (>= %.0f), recovered to %.1f mm (<= %.0f), WBC ok all ticks=%s" % (
    'PASS' if ok else 'FAIL', maxh * 1000, min_disp * 1000, finh * 1000, max_final * 1000, ok_all))
sys.exit(0 if ok else 1)
PY

cat "$OUT"
grep -q '^PASS' "$OUT"
