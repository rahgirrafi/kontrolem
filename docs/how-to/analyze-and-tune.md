# Analyze & tune a controller offline

**Goal:** design, validate, and stress a control law for your robot **before it
moves** — from a terminal, with no launch, no middleware, no simulator install.

The `kontrolem_setup` CLI (package `kontrolem_workbench`) is the design-time
side of the framework. It runs the *same* ROS-free core the runtime deploys —
the law you analyze is byte-identical to the law that runs — and it discovers
laws through the same plugin registry, so third-party controllers get the same
treatment as shipped ones.

```bash
source install/setup.bash   # the registry lives on the ament index
alias kontrolem_setup=$(ros2 pkg prefix kontrolem_workbench)/lib/kontrolem_workbench/kontrolem_setup
```

## 1 — What laws exist, and what are their knobs?

```bash
kontrolem_setup list
kontrolem_setup describe lpv
```

`describe` prints the law's live parameter schema — name, type, default,
meaning — straight from its `ParameterSpec`. This is the authoritative version
of the [parameter reference](../reference/controller-parameters.md).

## 2 — Start a config for your robot

```bash
kontrolem_setup new --urdf my_robot.urdf --law lqr -o my_controllers.yaml
```

Every parameter is present at its default with its description as a comment,
and `actuated_joints` is pre-filled from the URDF. Edit, then…

## 3 — Validate it without launching

```bash
kontrolem_setup lint my_controllers.yaml --urdf my_robot.urdf
```

`lint` catches, in about a second, everything that would otherwise surface at
`on_configure` (or worse, not at all):

- a typo'd parameter (`lpv.sched_nods`) — with a *did-you-mean* suggestion;
- a wrong type — ROS parameter typing is strict: `1` is an int, `1.0` a double;
- a mis-shaped structure (an LPV grid whose `sched_min` is shorter than its
  joint list), an unknown joint or contact frame, a wrong `q_ref` length;
- parameters for a law the config does not select (warning — legal, but worth
  knowing).

With `--urdf` it *dry-runs the law's actual construction* against your robot
model — the exact check the runtime performs, minus the robot. Run it in CI.

## 4 — Test the closed loop with test signals

```bash
kontrolem_setup analyze my_controllers.yaml --urdf my_robot.urdf \
    --signal release --csv run.csv
python3 $(ros2 pkg prefix kontrolem_workbench)/lib/kontrolem_workbench/plot_run.py run.csv
```

Signals: `release` (start displaced, regulate back — the classic step
response), `step` (setpoint jumps mid-run), `push` (a disturbance torque burst;
use `--push-joint`/`--push-mag`), `sine` (track a harmonic — laws that accept
Tracking only). The report gives the **shared empirical tier** every law gets:

```
settling time    0.502 s  (into the +/-0.02 band)
overshoot        0 %
steady-state err 0.0006
peak |tau|       30 Nm  (SATURATED 3.1% of ticks)
status ok        100 % of ticks  (worst margin -9.9e-07)
```

Note the last two lines — saturation fraction and worst trust margin are the
gauges a torque plot alone hides: a law that "works" at 100% saturation has no
authority left for the next disturbance.

## 5 — Read the law-specific pane

`analyze` also prints **the analysis appropriate to the specific law**:

| Law | Its pane |
|---|---|
| `lqr` | Closed-loop poles of `A−BK` at the pose (stability, damping, time constants) + the trust region. |
| `lqg` | Controller poles **and** estimator poles, and whether the estimator is actually faster. |
| `lpv` | Local stability at every grid node **and** at inter-node midpoints, plus the **interpolation gap** — how much worse the interpolated gain is than a freshly designed one at the same pose (the risk the point-wise design has no certificate for; refine the grid until it is small). |
| `mpc` | The plant's open-loop poles, prediction-window adequacy, and condensed-Hessian conditioning. |
| `qp` | Its **imposed error dynamics** (`s² + k_d s + k_p`): predicted settling printed next to the simulated one — a mismatch means the QP's constraints are biting — plus how much of `tau_max` just holding the pose consumes. |

Laws without a pane (including any third-party law) still get the full
simulated tier.

!!! warning "Evidence, not certificates"
    Eigenvalue analysis is **local** — valid at the analyzed pose. A passing
    simulation is evidence at *that* scenario. Neither is a global stability
    proof; treat them the way a control engineer treats a bench test.

## 6 — Record what actually ran

Set `provenance_dir: "runs"` in the deployment YAML and every `on_configure`
writes a manifest — every resolved parameter plus a hash of the URDF — so any
experiment (a thesis plot, a failed trial) can be traced to its exact
configuration:

```yaml
# runs/2026-07-31_14-02-11_manifest.yaml
stamp: 2026-07-31_14-02-11
urdf_hash: fnv1a64:d8f08ff46d60bf1f
parameters:
  control_law: lpv
  lpv.sched_nodes: [9, 9]
  ...
```

## Limits (v1)

`analyze` simulates **fixed-base** robots (the floating-base/contact tier is a
planned extension — `lint`, `describe`, `new`, and provenance already handle
floating-base configs). Panes exist for `lqr`/`lqg`/`lpv`/`mpc`/`qp`;
`wbc`/`kinematic_gait` currently get the shared tier.
