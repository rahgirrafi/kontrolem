#!/usr/bin/env bash
# End-to-end INDEPENDENT-PHYSICS validation (M6.1). Launch the cart-pole in Gazebo
# Fortress (gz-sim), driven by the SAME KontrolemController (LQR) + the SAME
# cart_pole_controllers.yaml used against the custom CartPoleSimSystem — only the
# plant differs (ign_ros2_control/IgnitionSystem). Once LQR is active, apply a
# one-shot torque disturbance to the pole via the world's ApplyLinkWrench system and
# verify LQR rejects it: the pole deviates, then returns to upright and the cart to
# centre, against Gazebo's own physics engine. Headless; standalone; hard timeout.
#
# Usage: e2e_cart_pole_gz.sh [torque] [run_s]
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
TORQUE="${1:-8.0}"        # one-shot pole torque (N·m for one 5 ms step)
RUN="${2:-50}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((RANDOM % 90 + 100))}"
OUT="$(mktemp)"; CAP="$(mktemp)"

cleanup() {
  pkill -9 -f "ign gazebo" 2>/dev/null; pkill -9 -f ruby 2>/dev/null
  pkill -9 -f robot_state_publisher 2>/dev/null; pkill -9 -f "ros2 launch" 2>/dev/null
  pkill -9 -f spawner 2>/dev/null; rm -f "$OUT" "$CAP"
}
trap cleanup EXIT

cat > "$CAP" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from kontrolem_msgs.msg import ControllerDiagnostics
dur = float(sys.argv[1]); out = sys.argv[2]
rows = []
class S(Node):
    def __init__(s):
        super().__init__('e2egz')
        s.create_subscription(ControllerDiagnostics, '/kontrolem_controller/diagnostics', s.cb, 10)
    def cb(s, m):
        if len(m.q) >= 2:
            rows.append((m.q[1], m.q[0]))
rclpy.init(); n = S(); t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.1)
with open(out, 'w') as f:
    if len(rows) < 100:
        f.write("FAIL nsamples=%d\n" % len(rows)); sys.exit()
    poles = [abs(r[0]) for r in rows]
    f.write("n=%d peak=%.4f final_pole=%.4f final_cart=%.4f\n" % (
        len(rows), max(poles), abs(rows[-1][0]), abs(rows[-1][1])))
PY

timeout "$RUN" ros2 launch kontrolem_bringup cart_pole_gz.launch.py >/dev/null 2>&1 &
sleep 20   # Gazebo startup + spawn + controller activation

python3 "$CAP" 14 "$OUT" &
GP=$!
sleep 3    # confirm the baseline hold before the kick
ign topic -t /world/cart_pole/wrench -m ignition.msgs.EntityWrench \
  -p "entity {name:\"cart_pole::pole\" type:LINK} wrench {torque {y: $TORQUE}}" >/dev/null 2>&1
wait $GP

read -r LINE < "$OUT"
echo "$LINE"
python3 - "$LINE" <<'PY'
import sys
line = sys.argv[1]
if line.startswith("FAIL") or "peak=" not in line:
    print("FAIL: no/low diagnostics — Gazebo may not have come up"); sys.exit(1)
d = dict(kv.split("=") for kv in line.split() if "=" in kv)
peak = float(d["peak"]); fp = float(d["final_pole"]); fc = float(d["final_cart"])
disturbed = 0.03 <= peak <= 0.6      # got kicked, but LQR kept it in-basin
recovered = fp < 0.03                 # pole back upright
centered = fc < 0.5                   # cart back near centre
ok = disturbed and recovered and centered
print("%s: Gazebo LQR — peak|pole|=%.3f (kicked, in-basin), final|pole|=%.4f, final|cart|=%.4f" % (
    "PASS" if ok else "FAIL", peak, fp, fc))
sys.exit(0 if ok else 1)
PY
