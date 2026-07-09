#!/usr/bin/env bash
# End-to-end MULTI-CONTROLLER SWITCH test: launch the cart-pole hosting LQR + MPC
# behind the Supervisor, let LQR balance, then publish a human switch command to
# MPC and verify (a) the ACTIVE law actually flips (diagnostics.control_law goes
# lqr -> mpc) and (b) the pole stays balanced THROUGH the switch (bumpless — no
# fall), ok on every tick. Standalone script with a hard timeout, like e2e_push.sh.
#
# Usage: e2e_switch.sh [launch] [max_pole_rad] [run_s]
set -o pipefail

LAUNCH="${1:-cart_pole_switch.launch.py}"
MAX_POLE="${2:-0.10}"   # pole must never exceed this (rad) — proves it never fell
RUN="${3:-28}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 90 + 100))}"
OUT="$(mktemp)"

cleanup() { pkill -9 -f ros2_control_node 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; pkill -9 -f spawner 2>/dev/null; rm -f "$OUT"; }
trap cleanup EXIT

timeout "$RUN" ros2 launch kontrolem_bringup "$LAUNCH" >/dev/null 2>&1 &

timeout "$RUN" python3 - "$MAX_POLE" > "$OUT" 2>/dev/null <<'PY'
import sys, rclpy
from rclpy.node import Node
from kontrolem_msgs.msg import ControllerDiagnostics
from std_msgs.msg import String
max_pole = float(sys.argv[1])
rows = []           # (control_law, |pole|, ok)
class S(Node):
    def __init__(s):
        super().__init__('e2eswitch')
        s.create_subscription(ControllerDiagnostics, '/kontrolem_controller/diagnostics', s.cb, 10)
        s.pub = s.create_publisher(String, '/kontrolem_controller/switch_controller', 10)
        s.sent = False
    def cb(s, m):
        if len(m.q) >= 2:
            rows.append((m.control_law, abs(m.q[1]), bool(m.ok)))
        # Once LGQ has clearly been running, command the switch to MPC (once).
        if not s.sent and len(rows) >= 300:
            s.pub.publish(String(data='mpc')); s.sent = True
rclpy.init(); n = S()
try: rclpy.spin(n)
except Exception: pass
if len(rows) < 400:
    print("FAIL: no/low diagnostics (%d samples) — launch may not have come up" % len(rows))
    sys.exit(1)
laws_before = set(l for l, _, _ in rows[:250])
final_law = rows[-1][0]
n_mpc = sum(1 for l, _, _ in rows if l == 'mpc')
# Balance THROUGH the switch: skip the startup catch (the sim starts the pole
# tipped at 0.15 rad and LQR settles it in the first ~1 s); the switch is
# commanded at ~sample 300, so from sample 250 on this isolates the handoff + MPC.
max_pole_seen = max(p for _, p, _ in rows[250:])
ok_all = all(o for _, _, o in rows)
switched = ('lqr' in laws_before) and (final_law == 'mpc') and (n_mpc > 100)
balanced = max_pole_seen <= max_pole
ok = switched and balanced and ok_all
print("%s: lqr->%s (mpc ticks=%d), max|pole|=%.4f rad (<= %.2f), ok all ticks=%s" % (
    'PASS' if ok else 'FAIL', final_law, n_mpc, max_pole_seen, max_pole, ok_all))
sys.exit(0 if ok else 1)
PY

cat "$OUT"
grep -q '^PASS' "$OUT"
