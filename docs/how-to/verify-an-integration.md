# How to verify an integration

> **For:** an integrator or developer who just built or changed something. **Assumes:** a built, sourced workspace. **Goal:** confirm the core is correct and the ROS runtime actually works, end-to-end.

There are two independent checks: **unit tests** (validate the ROS-free core numerics) and the **end-to-end smoke suite** (validate the full ros2_control runtime).

## Unit tests

The unit tests live in `kontrolem_model` and `kontrolem_controllers`. Because the tests link Pinocchio from the cmeel wheel, they need its libraries on the loader path — `colcon test` does **not** set that, so run the test binaries directly:

```bash
source /opt/ros/humble/setup.bash
P=$HOME/.local/lib/python3.10/site-packages/cmeel.prefix
export LD_LIBRARY_PATH="$P/lib:$P/lib64:$LD_LIBRARY_PATH"

cd <workspace>
for t in build/kontrolem_model/test_* build/kontrolem_controllers/test_*; do
  [ -x "$t" ] && { "$t" >/dev/null 2>&1 && echo "PASS $(basename $t)" || echo "FAIL $(basename $t)"; }
done
```

Every line should read `PASS`. These cover, for example, analytic-vs-finite-difference linearization, the contact dynamics, allocation-free `compute()`, and closed-loop stabilization of each controller.

> Running `colcon test` directly will report failures with `error while loading shared libraries: libboost_serialization…`. That's the missing loader path, **not** a real test failure — see [Fix the cmeel / libboost load error](fix-cmeel-library-errors.md).

## End-to-end smoke suite

The unit tests don't exercise the ros2_control runtime (interface wiring, the supervisor, the sim hardware). The smoke suite does: it launches each demo, watches a joint settle, and reports PASS/FAIL with hard timeouts.

```bash
source install/setup.bash
bash kontrolem_bringup/test/e2e_all.sh
```

Expected tail:

```
E2E SMOKE: ALL PASS
```

To run one demo only:

```bash
# e2e_smoke.sh <launch> <joint> <threshold> [settle_s] [run_s] [target]
bash kontrolem_bringup/test/e2e_smoke.sh cart_double_pole.launch.py pole1_joint 0.05
bash kontrolem_bringup/test/e2e_smoke.sh quad_stand.launch.py       knee_FL     0.2 9 14 -1.4
```

The `target` argument (default 0) is the value the watched joint should settle *to* — 0 for a regulator, a nominal posture value (e.g. `-1.4`) for the standing WBC.

> The smoke suite is a standalone script, not a `colcon` launch-test, on purpose. Full ros2_control launches can flake under CI/harness setups, and a hanging test is worse than a script you run deliberately — the reasoning is in [Explanation → Design decisions](../explanation/design-decisions.md).
