# Extending the framework

Three extension points, one file each. All of them follow the same rule:
register a class, and it appears automatically in the Python API, the YAML
specs, the CLI, and the wizard — no other edits.

## Add a controller

Create `state_space_control/controllers/my_ctrl.py`:

```python
from ..base import ControllerDesign, ControllerResult, register

@register('my_ctrl')
class MyController(ControllerDesign):
    def __init__(self, some_param=1.0):
        self.some_param = some_param

    def design(self, plant):
        K = ...                      # your synthesis
        return ControllerResult(name='my_ctrl', plant=plant, K=K)
```

then import it in `controllers/__init__.py`. Return either a static gain
`K` (control law `u = u_eq − K x`) **or** a dynamic output-feedback
controller as an LTI system from `y` to `u` (`controller=` argument);
`closed_loop()`, the benchmark, the response simulator, and the wizard all
handle both forms. Semantics to respect: every controller here is a
**regulator about the operating point** — it drives the deviation state to
zero, with `u_eq` applied as feedforward.

If your parameters need UI hints in the wizard, add an entry to the
assistant's `schemas.py`; otherwise the form is generated from your
`__init__` signature.

## Add an excitation

```python
from state_space_control.excitations import Excitation, register_excitation

@register_excitation('chirp')
class Chirp(Excitation):
    """Frequency sweep from f0 to f1 over the horizon."""

    PARAMS = [
        {'name': 'amplitude', 'default': 1.0},
        {'name': 'f0', 'default': 0.1, 'doc': 'start frequency [Hz]'},
        {'name': 'f1', 'default': 5.0, 'doc': 'end frequency [Hz]'},
    ]

    def __init__(self, amplitude=1.0, f0=0.1, f1=5.0):
        self.amplitude, self.f0, self.f1 = amplitude, f0, f1

    def sample(self, t):
        import numpy as np
        k = (self.f1 - self.f0) / t[-1]
        return self.amplitude * np.sin(
            2 * np.pi * (self.f0 + 0.5 * k * t) * t)
```

`PARAMS` drives the wizard's form; the class docstring becomes its help
text. In v1 all excitations enter as an **input disturbance at the plant
input** (`u = u_ctrl + d(t)`); the `injection` class attribute is
`'input'`, with `'reference'` and `'output'` reserved for future reference
tracking and measurement disturbances.

One special case worth knowing: a true impulse δ(t) cannot be sampled (a
one-sample pulse depends on the grid), so the built-in `impulse` is
realized as the exact LTI-equivalent state jump `x0 += B[:, ch] · area`.

## Add a renderer

A renderer turns sampled frames into pixels somewhere. Implement two
methods:

```python
from state_space_response_viz.renderers import TrajectoryRenderer

class CsvRenderer(TrajectoryRenderer):
    """Dump every rendered frame to CSV (a minimal 'screen')."""

    def __init__(self, path):
        self._f = open(path, 'w')

    def setup(self, traj):
        self._names = list(traj.joint_names)
        self._f.write('t,' + ','.join(self._names) + '\n')

    def render(self, frame):
        self._f.write(f"{frame.t}," + ','.join(
            str(frame.joint_positions[n]) for n in self._names) + '\n')
```

and add an instance to the player node's renderer list. The engine fans
out every frame to all active renderers, so RViz + your renderer run
simultaneously. `frame.base_pose` is `None` for fixed-base trajectories —
render the base only when it is present. A video exporter is the same
pattern driven by a synthetic fixed-step clock instead of wall time.

## Add a trajectory producer

Anything that can fill the arrays of the
{doc}`RobotTrajectory schema <trajectory_format>` is a producer — a
nonlinear simulator, a rosbag importer, a hardware logger. Build the
dataclass, call `validate()`, `save_npz()`. Set `meta['source']` to
identify yourself and include whatever reproducibility keys you can. Your
file then plays in RViz and the wizard, gets event annotations from
`annotate_events()` (a pure function over the format), and participates in
future comparison/report tooling — with no changes to any existing code.

## Conventions your extension must keep

- **Absolute joint positions** in trajectories; `u` in deviation form.
- **All movable joints present** in every frame — omitting unactuated
  joints leaves stale TF in RViz.
- **Never clamp data** to hide linear-model artifacts; annotate with
  events instead.
- Heavy imports (scipy, control, ROS message types) **lazy** — module
  import must stay cheap, and the registries import every plugin module.

## Testing

Each package has a headless pytest suite (`test/`). Extensions should
follow the pattern: controllers get a closed-loop stability test on the
analytic pendulum plants in `test_controllers.py`; excitations get a
shape/values test; renderers are tested against a hand-built trajectory
with a fake node (see `test_renderers.py`) — proving they work without
any simulator.
