#!/usr/bin/env bash
# Run the e2e smoke test across the demos and report an overall PASS/FAIL.
# Guards the ros2_control runtime integration (which the unit tests do NOT cover)
# for LQR, LQG and QP on three robots. Run after building + sourcing the workspace:
#   source install/setup.bash && bash kontrolem_bringup/test/e2e_all.sh
HERE="$(cd "$(dirname "$0")" && pwd)"
SMOKE="$HERE/e2e_smoke.sh"

# each row: launch | joint | threshold [| settle | run | target]  (see e2e_smoke.sh)
CASES=(
  "cart_pole.launch.py|pole_joint|0.05"              # LQR, cart-pole
  "cart_pole_lqg.launch.py|pole_joint|0.05"          # LQG, output feedback
  "cart_double_pole.launch.py|pole1_joint|0.05"      # LQR, hard 3-DoF benchmark
  "arm2.launch.py|shoulder_joint|0.05"               # QP task-space, 2-DoF arm
  "quad_stand.launch.py|knee_FL|0.2|9|14|-1.4"       # QP-WBC, quadruped standing
)

fails=0
for row in "${CASES[@]}"; do
  IFS='|' read -ra A <<< "$row"
  printf '  %-32s %-16s ' "${A[0]}" "${A[1]}"
  if bash "$SMOKE" "${A[@]}" | tail -1; then :; else fails=$((fails+1)); fi
done

# Push-recovery has its own metric (base deviates during the push, then returns —
# read from ControllerDiagnostics), so it uses e2e_push.sh rather than the joint-
# settle smoke test.
printf '  %-32s %-16s ' "quad_push.launch.py" "push-recovery"
if bash "$HERE/e2e_push.sh" | tail -1; then :; else fails=$((fails+1)); fi

# Multi-controller switch: LQR -> MPC on a human command, bumpless (custom metric:
# the active law flips while the pole stays balanced), so it uses e2e_switch.sh.
printf '  %-32s %-16s ' "cart_pole_switch.launch.py" "ctrl-switch"
if bash "$HERE/e2e_switch.sh" | tail -1; then :; else fails=$((fails+1)); fi

echo
if [ "$fails" -eq 0 ]; then echo "E2E SMOKE: ALL PASS"; exit 0
else echo "E2E SMOKE: $fails FAILED"; exit 1; fi
