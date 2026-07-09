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

echo
if [ "$fails" -eq 0 ]; then echo "E2E SMOKE: ALL PASS"; exit 0
else echo "E2E SMOKE: $fails FAILED"; exit 1; fi
