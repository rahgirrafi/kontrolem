# Architecture

## Design commitments

The framework is built around three commitments:

1. **One canonical interchange format.** Every module that touches motion
   data speaks {doc}`RobotTrajectory <trajectory_format>`; producers and
   consumers never know about each other.
2. **Analytic over numeric.** The plant matrices come from Pinocchio's
   analytic derivatives (`computeABADerivatives`), not finite differences;
   the impulse excitation is realized as the exact LTI state jump, not a
   sampled pulse.
3. **The exported artifact is byte-wise the thing that produced the
   pixels.** The wizard writes `model.yaml` / `design.yaml` /
   `trajectory.npz` to a staging directory and runs the *same loaders the
   CLIs use*, so an exported bundle provably reproduces what was on screen.

## The canonical-format philosophy

`RobotTrajectory` is the framework's interchange format, **not an export
by-product**:

```text
 PRODUCERS (any of)                                CONSUMERS (any of)
 ─────────────────                                 ─────────────────
 linear state-space sim (today) ┐              ┌─▶ web viewer (three.js)
 nonlinear simulators (future) ─┤              ├─▶ RViz playback (ROS 2)
 MuJoCo / Pinocchio dynamics  ──┼▶ Robot     ──┼─▶ benchmark / comparison playback
 Gazebo runs (future)          ─┤  Trajectory  ├─▶ report generation
 rosbag2 / real-robot logs    ──┘  (.npz)      ├─▶ video export (future)
                                               └─▶ Foxglove / Unity (future)
```

A consumer requires only a *valid trajectory*; nothing in the visualization
subsystem imports the simulator, the controller registry, or Pinocchio.
Adding a producer (say, a rosbag importer) or a consumer (say, a video
exporter) touches zero existing code. The test suites enforce this: the
playback package's tests run against hand-built trajectories, never
against a simulation.

## The processing pipeline

Four layers, each with a single responsibility and its own tests:

```text
┌────────────────────────────────────────────────────────────────────────────┐
│ Layer 1 — TRAJECTORY PRODUCTION       (state_space_control, pure math)     │
│  excitations.py: Excitation plugin registry (step/impulse/ramp/sine/       │
│                  custom/zero — extensible to references, noise, forces)    │
│  simulation.py: simulate_response(ControllerResult, excitation, x0, t)     │
│                 → deviation-space result → RobotTrajectory (via q_eq       │
│                   reconstruction + metadata + event annotation)            │
├────────────────────────────────────────────────────────────────────────────┤
│ Layer 2 — CANONICAL ARTIFACT          (trajectory.py, numpy-only)          │
│  RobotTrajectory {t, base_pose?, base_twist?, q, qd, u, events, meta}      │
│  + npz round-trip, schema versioning, validation                           │
├────────────────────────────────────────────────────────────────────────────┤
│ Layer 3 — SAMPLING                    (trajectory.py)                      │
│  FrameSampler: "what does the robot look like at time t?"                  │
│  → RobotFrame {t, base_pose?, joint_positions, joint_velocities, u}        │
│  Interpolation strategy lives here and only here                           │
├──────────────────────────────┬─────────────────────────────────────────────┤
│ Layer 4a — WEB PLAYBACK      │ Layer 4b — ROS 2 PLAYBACK                   │
│ PlaybackClock + FrameSampler │ PlaybackClock + FrameSampler (Python)       │
│ (JS twins) → ThreeJSRenderer │ → TrajectoryRenderer interface              │
│ + time-cursor plots +        │   RVizRenderer: JointState (+ TF for        │
│ event-marked timeline        │   floating bases) behind the API            │
└──────────────────────────────┴─────────────────────────────────────────────┘
```

## Playback = clock × sampler × renderer

Playback decomposes into three orthogonal pieces, implemented once in
Python (`state_space_response_viz`) and once in JavaScript (the wizard) —
same states, same transition rules, so web and RViz playback behave
identically:

`PlaybackClock`
: Answers *"what simulation time is it?"* — play/pause/seek/speed/reset.
  Every mutation rebases a `(wall_anchor, sim_anchor)` pair, so a speed
  change never makes the cursor jump. Pose is always a function of
  monotonic wall time — never an `index++` per frame — so timer jitter and
  dropped frames cannot desynchronize anything.

`FrameSampler`
: Answers *"what does the robot look like at that time?"* — owns
  interpolation entirely (linear default, zero-order hold available, slerp
  for base quaternions). A 60 Hz rclpy timer, a browser
  `requestAnimationFrame` loop and a future fixed-30-fps video exporter
  all call the identical `frame_at(t)`.

`TrajectoryRenderer`
: Answers *"put that frame on a screen"*. The RViz implementation
  publishes `sensor_msgs/JointState` (and a world→base TF when the
  trajectory carries a base pose); the web implementation calls
  `setJointValue` on the three.js model. Those transport details stay
  hidden behind `setup(traj)` / `render(frame)`, and the playback engine
  drives a *list* of renderers.

In the wizard, one clock and one sampler drive the robot pose, every plot
cursor, and the scrub bar from a single animation callback — the
synchronization between the 3D animation and the analysis plots is
**structural**, not incidental.

## Deviation coordinates: the one dangerous spot

The plant state is in deviation coordinates, `x = [q − q_eq; q̇]`, but a
renderer needs **absolute** joint positions. The reconstruction
`q(t) = q_eq + δq(t)` happens in exactly one function —
`state_space_control.simulation.to_robot_trajectory` — and nowhere else.
Everything downstream of that function (the npz file, both renderers, the
plots) sees absolute joint space only. A unit test pins the invariant: a
zero trajectory renders the *equilibrium* pose, not the zero pose.

Control inputs are stored the other way: `u` in a trajectory is the
deviation `u − u_eq`, with `u_eq` recorded in the metadata — one
convention, stated in the {doc}`format spec <trajectory_format>`, so
producers can never disagree silently.

## Honesty annotations

A linearized response is only as valid as the linearization. Rather than
silently clamping or hiding artifacts, the simulator *annotates* the
trajectory with time-stamped events that every consumer surfaces
(warning banners, shaded plot spans, timeline ticks, a ROS topic):

- `linear_validity` — a joint's excursion from `q_eq` exceeds a threshold;
  beyond it the linear model is fiction.
- `limit_violation` — the linear simulation ignores URDF joint limits;
  violating spans are flagged, never clamped (clamping would misrepresent
  the model).
- `instability` — unstable closed loops are simulated (divergence is worth
  *seeing*) but time-capped before the numbers overflow the renderer.
- `settling_time`, `overshoot_peak` — classical step-response landmarks.

## Plugin registries

Three extension points share the same pattern — a class decorated with a
`register` function, discovered automatically by the CLI, the YAML specs,
and the wizard (see {doc}`extending`):

| Registry | Base class | Built-ins |
|---|---|---|
| Controllers | `state_space_control.base.ControllerDesign` | `lqr`, `lqg`, `hinf_mixsyn`, `hinf`, `pid` |
| Excitations | `state_space_control.excitations.Excitation` | `step`, `impulse`, `ramp`, `sine`, `custom`, `zero` |
| Renderers | `state_space_response_viz.renderers.TrajectoryRenderer` | `RVizRenderer` (+ `ThreeJSRenderer` in JS) |
