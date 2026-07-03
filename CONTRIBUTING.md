# Contributing

Thanks for your interest in improving this project! Contributions of all
kinds are welcome: bug reports, documentation fixes, new controller
synthesis methods, additional robot examples, and test coverage.

This document explains how to get set up, the conventions the codebase
follows, and how to submit changes. By participating you agree to abide by
our [Code of Conduct](CODE_OF_CONDUCT.md).

## Ways to contribute

- **Report a bug** — open a [GitHub issue](../../issues) with a minimal
  reproducer: the URDF (or a link to it), the design YAML, the command you
  ran, and the full traceback or unexpected output.
- **Request a feature** — open an issue describing the use case. For a new
  controller synthesis method, say which plant class it targets and cite the
  method if it comes from the literature.
- **Ask a question** — open an issue with the `question` label. There are no
  bad questions; if something in the docs was unclear, that is a
  documentation bug worth fixing.
- **Send a pull request** — see below.

## Development setup

This is a [colcon](https://colcon.readthedocs.io/) / ROS 2 workspace. You
need ROS 2 Humble (or newer) and Python ≥ 3.10.

```bash
# 1. Clone into a workspace `src/` directory
mkdir -p ~/ws/src && cd ~/ws/src
git clone <your-fork-url> .

# 2. Install Python dependencies
pip install pin numpy scipy pyyaml flask   # core
pip install control slycot                 # optional: H-infinity synthesis

# 3. Build and source
cd ~/ws
colcon build
source install/setup.bash
```

The `pin` package is the PyPI distribution of
[Pinocchio](https://github.com/stack-of-tasks/pinocchio); do **not**
`pip install pinocchio` (that is an unrelated project). `slycot` needs a
Fortran compiler and BLAS/LAPACK (`apt install gfortran liblapack-dev
libblas-dev`) and is only required for the H∞ controller.

## Running the tests

Most of the test suite is pure Python and runs without a ROS graph:

```bash
# Core libraries — fast, no ROS required
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
    src/urdf_state_space/test/ \
    src/state_space_control/test/ \
    src/state_space_setup_assistant/test/ \
    --ignore=src/state_space_control/test/test_trajectory.py
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` prevents the `ament_*` pytest plugins
from hijacking collection when you run outside a sourced ROS environment.

The packages that publish/subscribe ROS messages (`state_space_response_viz`,
the trajectory ROS bridge, and the `ament_*` lint tests) are exercised by
`colcon test` inside a sourced workspace:

```bash
colcon test --packages-select state_space_response_viz
colcon test-result --verbose
```

Continuous integration (`.github/workflows/ci.yml`) runs both paths on every
push and pull request: a fast core-library job and a full ROS 2 Humble
`colcon test` job.

## Adding a new controller (plugin registry)

Controllers are discovered through a decorator-based registry, so you can add
a synthesis method without touching the CLI or the setup assistant. Create a
module under `src/state_space_control/state_space_control/controllers/` and
register a design class:

```python
from state_space_control.base import ControllerDesign, register

@register("my_method")
class MyMethodDesign(ControllerDesign):
    """One-line description shown by `ss_design --list`."""

    def design(self, plant, **params):
        # plant.A, plant.B, plant.C, plant.D are numpy arrays.
        # Return a ControllerResult (see base.py for the fields).
        ...
```

Import the module in the `controllers/__init__.py` so the `@register` call
runs at import time. Then it is immediately available as
`make_controller("my_method", ...)`, from the `ss_design` CLI, and in the web
setup assistant. Please add a test under
`src/state_space_control/test/` and a design YAML under
`src/state_space_control/examples/`.

## Trajectory / response file format

Closed-loop responses are exchanged as `RobotTrajectory` objects serialized
to `.npz`. If you add a producer or consumer, keep the schema consistent with
`state_space_control.trajectory.RobotTrajectory` (see its docstring for the
array names and shapes) so files stay interoperable with the RViz playback
tool.

## Pull request checklist

1. Fork the repository and create a topic branch off `main`.
2. Make focused commits with clear messages.
3. Add or update tests for the behaviour you changed.
4. Run the relevant test suite locally and make sure it passes.
5. Update the affected package `README.md` and, if user-facing, the
   top-level `README.md`.
6. Open the pull request against `main` and describe the motivation and
   approach. Link any related issue.

Maintainers aim to respond to issues and pull requests within a couple of
weeks. Thank you for contributing!
