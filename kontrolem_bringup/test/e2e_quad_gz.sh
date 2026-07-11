#!/usr/bin/env bash
# End-to-end WBC-under-real-contact validation (M6.2). The SAME KontrolemController
# (control_law: wbc) + the SAME quad_stand_controllers.yaml that stand the quadruped
# against FloatingContactSimSystem (pinned feet) now stand it against Gazebo
# Fortress's OWN contact/friction solver: joints via ign_ros2_control/IgnitionSystem,
# base pose/twist + contact via kontrolem_gz/GzBaseStateSystem (read from the ECM).
#
# Sequence: spawn WELDED to a static anchor (a DetachableJoint freezes the whole
# model in DART — the "hold until commanded" analogue), WBC activates against the
# frozen robot, DETACH hands it a clean nominal state, assert it holds the stance
# on real contact, then shove the base (one-shot ApplyLinkWrench) and assert it
# recovers. Headless; standalone; hard timeout.
#
# Usage: e2e_quad_gz.sh [push_N] [run_s]
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PUSH="${1:-3000.0}"       # one-shot lateral force on the base (N, for one 2 ms step)
RUN="${2:-60}"
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
tag, dur, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
rows = []
class S(Node):
    def __init__(s):
        super().__init__('e2equad')
        s.create_subscription(ControllerDiagnostics, '/kontrolem_controller/diagnostics', s.cb, 10)
    def cb(s, m):
        if len(m.q) >= 3:
            rows.append((m.q[1], m.q[2], 1.0 if m.ok else 0.0))
rclpy.init(); n = S(); t0 = time.time()
while time.time() - t0 < dur:
    rclpy.spin_once(n, timeout_sec=0.05)
with open(out, 'a') as f:
    if len(rows) < 100:
        f.write("%s FAIL n=%d\n" % (tag, len(rows)))
    else:
        ys = [r[0] for r in rows]; zs = [r[1] for r in rows]
        okr = sum(r[2] for r in rows) / len(rows)
        f.write("%s n=%d y_peak=%.4f y_final=%.4f z_min=%.4f z_final=%.4f ok_rate=%.4f\n" % (
            tag, len(rows), max(ys, key=abs), ys[-1], min(zs), zs[-1], okr))
PY

timeout "$RUN" ros2 launch kontrolem_bringup quad_gz.launch.py >/dev/null 2>&1 &
sleep 13   # gazebo + spawn + weld + WBC activation (robot frozen at nominal)

# Hand the WBC the robot: detach the startup weld, let it settle into stance.
ign topic -t /quadruped/detach -m ignition.msgs.Empty -p "" >/dev/null 2>&1
sleep 3
python3 "$CAP" STANCE 3 "$OUT"

# Shove the base laterally (one-shot => one 2 ms physics step, deterministic).
ign topic -t /world/quadruped/wrench -m ignition.msgs.EntityWrench \
  -p "entity {name:\"floating_quadruped::base_link\" type:LINK} wrench {force {y: $PUSH}}" >/dev/null 2>&1
python3 "$CAP" PUSH 10 "$OUT"

cat "$OUT"
python3 - "$OUT" <<'PY'
import sys
lines = {l.split()[0]: l for l in open(sys.argv[1]) if l.strip()}
def vals(tag):
    d = dict(kv.split("=") for kv in lines[tag].split()[1:] if "=" in kv)
    return {k: float(v) for k, v in d.items()}
if "STANCE" not in lines or "PUSH" not in lines or "FAIL" in lines.get("STANCE","F")+lines.get("PUSH","F"):
    print("FAIL: incomplete data — Gazebo/WBC may not have come up"); sys.exit(1)
s, p = vals("STANCE"), vals("PUSH")
stance = 0.25 <= s["z_min"] and s["z_final"] <= 0.32 and abs(s["y_final"]) < 0.02 and s["ok_rate"] > 0.99
pushed = 0.02 <= abs(p["y_peak"]) <= 0.15     # visibly shoved, but in-basin
recovered = abs(p["y_final"]) < 0.02 and 0.25 <= p["z_final"] <= 0.32 and p["ok_rate"] > 0.99
ok = stance and pushed and recovered
print("%s: Gazebo WBC — stance(z=%.3f, ok=%.3f)=%s push(y_peak=%.3f)=%s recovery(y=%.3f z=%.3f ok=%.3f)=%s" % (
    "PASS" if ok else "FAIL", s["z_final"], s["ok_rate"], stance,
    p["y_peak"], pushed, p["y_final"], p["z_final"], p["ok_rate"], recovered))
sys.exit(0 if ok else 1)
PY
