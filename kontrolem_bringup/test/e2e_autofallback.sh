#!/usr/bin/env bash
# End-to-end AUTOMATIC fail-forward test: launch the cart-pole hosting LQR (primary,
# modest trust region) + MPC (fallback) with auto_fallback on. The sim shoves the
# pole a few seconds in, driving the state out of LQR's region. Verify that WITH NO
# human command the active law flips lqr -> mpc on its own (the Supervisor acting on
# the trust predicate) and the pole is caught and settles. Standalone, hard timeout.
#
# Usage: e2e_autofallback.sh [launch] [max_pole_rad] [run_s]
set -o pipefail

LAUNCH="${1:-cart_pole_autofallback.launch.py}"
MAX_POLE="${2:-0.60}"   # observed swing ~0.24 rad during failover; 0.60 gives headroom yet is well under pi/2 (never fell)
RUN="${3:-35}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 90 + 100))}"
OUT="$(mktemp)"

cleanup() { pkill -9 -f ros2_control_node 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; pkill -9 -f spawner 2>/dev/null; rm -f "$OUT"; }
trap cleanup EXIT

timeout "$RUN" ros2 launch kontrolem_bringup "$LAUNCH" >/dev/null 2>&1 &

timeout "$RUN" python3 - "$MAX_POLE" > "$OUT" 2>/dev/null <<'PY'
import sys, rclpy
from rclpy.node import Node
from kontrolem_msgs.msg import ControllerDiagnostics
max_pole = float(sys.argv[1])
rows = []   # (control_law, |pole|)
class S(Node):
    def __init__(s):
        super().__init__('e2eauto')
        s.create_subscription(ControllerDiagnostics, '/kontrolem_controller/diagnostics', s.cb, 10)
    def cb(s, m):
        if len(m.q) >= 2:
            rows.append((m.control_law, abs(m.q[1])))
rclpy.init(); n = S()
try: rclpy.spin(n)
except Exception: pass
if len(rows) < 500:
    print("FAIL: no/low diagnostics (%d samples) — launch may not have come up" % len(rows))
    sys.exit(1)
laws_early = set(l for l, _ in rows[:250])           # before the shove: LQR
final_law = rows[-1][0]
n_mpc = sum(1 for l, _ in rows if l == 'mpc')
# Find where MPC first takes over (the automatic failover), and check balance after.
first_mpc = next((i for i, (l, _) in enumerate(rows) if l == 'mpc'), None)
max_pole_after = max((p for _, p in rows[first_mpc:]), default=9.9) if first_mpc is not None else 9.9
final_pole = rows[-1][1]
auto_failed_over = (laws_early == {'lqr'}) and (final_law == 'mpc') and (n_mpc > 100)
survived = max_pole_after <= max_pole            # caught the shove, never fell
recovered = final_pole < 0.05                    # settled back upright
ok = auto_failed_over and survived and recovered
print("%s: lqr->%s auto (mpc ticks=%d), max|pole| after failover=%.3f rad (<= %.2f), final|pole|=%.4f" % (
    'PASS' if ok else 'FAIL', final_law, n_mpc, max_pole_after, max_pole, final_pole))
sys.exit(0 if ok else 1)
PY

cat "$OUT"
grep -q '^PASS' "$OUT"
