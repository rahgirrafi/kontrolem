# state_space_control — synthesis toolbox

Modular **controller-synthesis toolbox** for linear state-space plants —
the design companion to {doc}`urdf_state_space`. It also hosts the
framework-wide plumbing that everything else consumes: the canonical
{doc}`trajectory format <../trajectory_format>`, the excitation registry,
and the closed-loop response simulator.

## Controllers

| Name | Type | Needs |
|---|---|---|
| `lqr` | static gain `K` | numpy + scipy |
| `lqg` | dynamic (Kalman filter + LQR) | numpy + scipy |
| `hinf_mixsyn` | dynamic, mixed-sensitivity H∞ | python-control + slycot |
| `hinf` | dynamic, general H∞ | python-control + slycot |
| `pid` | dynamic, per-joint parallel PID | numpy + scipy |

```python
from state_space_control import Plant, make_controller

plant = Plant.from_npz('model.npz')        # exported by urdf2ss
result = make_controller('lqr', Q=[1, 10, 10, 1, 1, 1], R=0.1).design(plant)
print(result.summary())
result.save_npz('controller.npz')
```

Weights accept a scalar (`q·I`), a flat list (diagonal), or a full matrix;
H∞ weights accept a scalar or `{num: [...], den: [...]}` transfer-function
coefficients.

### Controller semantics

A `ControllerResult` holds exactly one of:

- `K` — static state-feedback gain, control law `u = u_eq − K x`;
- `controller` — dynamic output-feedback LTI system from measured `y` to
  `u`, sign convention already absorbed (the closed loop is literally
  `u = controller(y)`).

`closed_loop()`, `closed_loop_poles()` and `is_stable()` work for both
forms — this is the controller-agnostic hook the whole framework builds
on. Every design here is a **regulator about the operating point**
(including the PID — it is *not* a setpoint tracker), with `u_eq` applied
as feedforward on the real robot.

Two synthesis caveats surfaced honestly rather than hidden:

- `hinf_mixsyn` can legitimately fail on plants with imaginary-axis poles
  (e.g. a free azimuth joint has a structural pole at the origin) — the
  slycot rank-condition error is reported with a hint, and runaway γ
  iterations are killed by a subprocess timeout instead of hanging.
- Naive per-joint PID on a coupled MIMO plant may not stabilize — the
  benchmark exists to show exactly that.

## CLI

```bash
ros2 run state_space_control ss_design plant.npz lqr_design.yaml -o controller.npz
```

with specs like:

```yaml
controller: lqr
params:
  Q: [1, 10, 10, 1, 1, 1]
  R: 0.1
```

## Excitations and response simulation

The excitation registry (`step`, `impulse`, `ramp`, `sine`, `custom`,
`zero`) feeds the closed-loop simulator; both are consumed by the wizard's
Response step and usable standalone:

```python
from state_space_control.excitations import make_excitation
from state_space_control.simulation import simulate_response, to_robot_trajectory

sim = simulate_response(result, make_excitation('sine', amplitude=2.0, freq_hz=0.5))
traj = to_robot_trajectory(ss, result, sim)   # absolute joint space + events + meta
traj.save_npz('trajectory.npz')
```

`simulate_response` works through `closed_loop()`, so it is
controller-agnostic; excitations enter as an input disturbance
(`u = u_ctrl + d(t)`), initial-condition experiments use `x0`, unstable
loops are simulated but time-capped. `to_robot_trajectory` is the *only*
place deviation coordinates become absolute joint positions, and it
attaches the reproducibility metadata and honesty events described in
{doc}`../architecture`.

## Analysis helpers

{mod}`state_space_control.analysis` provides `damping_report`,
`step_response`, and `settling_time` — the primitives behind the wizard's
benchmark table.

## API reference

{doc}`../api/state_space_control`
