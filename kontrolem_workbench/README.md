# kontrolem_workbench

## Purpose

The offline **design workbench** (M18): the `kontrolem_setup` CLI a control
engineer uses *before* the robot moves. It turns M16's config-as-data into
working tooling — discover the installed control laws and their parameter
schemas (`list` / `describe`), validate and scaffold deployment YAMLs
(`lint` / `new`), and analyze a configured law offline (`analyze`): closed-loop
test-signal simulation with time-domain metrics for **every** law, plus
law-specific analysis panes (closed-loop eigenvalues for `lqr`/`lqg`, the LPV
interpolation-gap check, MPC conditioning/horizon adequacy, the QP's
imposed-dynamics prediction). `docs` emits the parameter tables as markdown.
Because the engine is the same ROS-free core the runtime deploys, the law you
analyze is byte-identical to the law that runs.

## Dependencies and build instructions

Depends on `kontrolem_control`, `kontrolem_controllers`, `kontrolem_model`
(the ROS-free core), `pluginlib` (discovery, CLI only), and `yaml-cpp` (via
`yaml_cpp_vendor`). Build from the workspace root:

```bash
colcon build --packages-up-to kontrolem_workbench \
  --cmake-args -DCMAKE_PREFIX_PATH="$HOME/.local/lib/python3.10/site-packages/cmeel.prefix"
```

Run with the workspace sourced (the plugin registry lives on the ament index);
the cmeel Pinocchio libs must be on `LD_LIBRARY_PATH` (see
`docs/reference/build-and-environment.md`). The optional `plot_run.py` needs
matplotlib.

## Relation to other packages

- **`kontrolem_control`** — supplies the `ControllerFactory` / `ParameterSpec`
  contract everything here consumes, and the base class the pluginlib
  discovery is keyed on.
- **`kontrolem_controllers`** — the shipped laws the panes analyze; the
  analyzers downcast their `*Synthesis` artifacts (`LqrSynthesis` etc.).
- **`kontrolem_model`** — the dynamics engine: `linearize` for the eigenvalue
  panes, `aba`/`integrate` for the simulator, `gravity_torque` for equilibria.
- **`kontrolem_controller_plugins`** (+ any third-party plugin package) — what
  discovery actually finds; a third-party law gets `list`/`describe`/`lint`/
  `new`/`analyze` (shared tier) with no workbench changes.
- **`kontrolem_ros2_control`** — the deployment counterpart: `lint` replicates
  its `on_configure` validation offline, and its `provenance_dir` manifest is
  the runtime half of the workbench's reproducibility story.

## Position in the complete architecture

A **design-time L4 consumer** of the L1–L3 core, beside (not inside) the
runtime. It is the payoff `architecture.md` promised for the ROS-free core
("the same core can back a design-time tool"): only the thin CLI touches
pluginlib; the engine (`libworkbench`: codec, lint, scaffold, simulator,
metrics, analyzers) is plain C++ against the core and fully offline-testable.
The L1–L3 ROS-free litmus is untouched.

## Intended use

Integrators bringing Kontrol'Em to a new robot (scaffold → lint → analyze →
launch), CI pipelines (lint every config in the repo), and experiment hygiene
(tune against the panes, record runs with `provenance_dir`). It is a bench
instrument, not a runtime component — nothing here runs in the control loop.

## How to use it

```bash
source install/setup.bash
CLI=$(ros2 pkg prefix kontrolem_workbench)/lib/kontrolem_workbench/kontrolem_setup

$CLI list                                       # what laws are installed?
$CLI describe wbc                               # a law's schema
$CLI new --urdf robot.urdf --law lqr -o c.yaml  # starter config
$CLI lint c.yaml --urdf robot.urdf              # validate offline (CI-able)
$CLI analyze c.yaml --urdf robot.urdf \
     --signal release --csv run.csv             # simulate + metrics + law pane
$CLI docs                                       # markdown parameter tables
```

See `docs/how-to/analyze-and-tune.md` for the walkthrough. Offline tests:
`test_spec_format`, `test_lint`, `test_simulator`, `test_analyzers` (no ROS
needed); `test_discovery` needs the sourced workspace.
