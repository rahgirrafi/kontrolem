# state_space_setup_assistant — the web wizard

A MoveIt2-Setup-Assistant-style **web wizard** for the whole pipeline:
load a URDF, validate it for control use, pick the operating point (with
an automatic equilibrium finder), linearize, design a controller, watch
the closed-loop response on the robot model, benchmark candidates, and
export a reproducible bundle.

```bash
ros2 run state_space_setup_assistant ss_setup_assistant \
    --urdf package://kontrolem_example_robots/urdf/cart_double_inverted_pendulum.urdf
```

Then open <http://127.0.0.1:8060/> (a tab opens automatically unless
`--no-browser`). **One browser tab at a time** — the server keeps a single
wizard session.

The frontend is Flask + vendored three.js/urdf-loader: no build toolchain,
no CDN at runtime, fully self-hosted. All numerical work happens
server-side through the same APIs the CLIs use.

## The eight steps

1. **Robot & validation** — load by path or `package://` URI (arg-less
   xacro works). The URDF is checked immediately: positive masses,
   positive-definite inertia + triangle inequality, disconnected links,
   missing limits, `<mimic>`/continuous joints, Pinocchio build. Errors
   block the wizard; warnings don't.
2. **Joints & operating point** — actuated checkboxes, per-joint damping,
   `q_eq` sliders (clamped to URDF limits) driving the 3D view live.
   *Auto-compute equilibrium* solves for the unactuated joints so they
   need no holding torque (`scipy.optimize.least_squares` on the gravity
   residual, indexed through Pinocchio's `idx_q`/`idx_v`).
3. **Outputs** — which joint positions (and optionally velocities) appear
   in the measurement vector `y`.
4. **Linearize** — writes `model.yaml` and runs it through
   `urdf_state_space.build_from_yaml`, so the exported file is
   byte-identical to what produced the on-screen `A/B/C/D`, pole map,
   controllability/observability badges and equilibrium warnings.
5. **Controller** — any design in the registry (`lqr`, `lqg`, `pid`,
   `hinf_mixsyn`, …); the spec goes to `design.yaml` and is read back
   exactly like `ss_design` does. New plugins appear automatically.
6. **Response** — simulate the closed loop under a test excitation and
   *watch the robot respond*: the 3D view animates the trajectory while a
   time cursor sweeps the joint-position and control-effort plots — one
   playback clock drives everything, so they can't drift apart. Transport
   bar with play/pause/reset, 0.25×–2× speed, drag-to-seek (scrub bar or
   directly on a plot). Honesty annotations appear as chips and plot
   markers: linear-validity excursions, URDF limit violations,
   instability, settling time. The full-resolution trajectory is saved as
   `trajectory.npz` for RViz playback.
7. **Benchmark** — synthesize several controllers and compare them on
   **one shared simulation grid** (same integrator, same units): settling
   time, overshoot, control effort ∫u²dt, peak sensitivity Ms, gain/phase
   margins (SISO). Failed syntheses become table rows, not crashes.
8. **Export** — `model.yaml`, `design.yaml`, `plant.npz`,
   `controller.npz`, `validation.md`, `benchmark.md` (+ traces),
   `trajectory.npz`, and `summary.txt` with copy-pasteable reproduction
   commands.

## Reproducibility guarantee

The linearize and design endpoints deliberately go *through the YAML
files*: they write `model.yaml`/`design.yaml` to a staging directory and
run the same loaders the CLIs use. Whatever you export provably reproduces
the numbers you saw:

```bash
ros2 run urdf_state_space urdf2ss model.yaml
ros2 run state_space_control ss_design plant.npz design.yaml -o controller.npz
ros2 launch state_space_response_viz view_response.launch.py trajectory:=trajectory.npz urdf:=…
```

## Notes and limitations

- Meshes are served from package share directories, so the workspace must
  be **built and sourced** for textured 3D; without it the wizard still
  works (frames only).
- Single operating point per run; multi-point/gain scheduling is future
  work (the export layout is one directory per operating point to make
  that bolt on).
- Continuous joints: `q_eq` unsupported (Pinocchio `nq=2`); mimic joints
  are ignored by Pinocchio — both flagged by validation.
- The server binds 127.0.0.1 by default; the mesh route is
  realpath-confined to package share directories.

## API reference

{doc}`../api/state_space_setup_assistant` — including the HTTP endpoints
(`create_app`) if you want to script the wizard.
