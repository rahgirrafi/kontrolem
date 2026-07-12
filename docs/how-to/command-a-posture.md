# Command the Go2's posture

**Goal:** make the standing Go2 *move its whole body* — squat, sway, tilt, yaw-twist —
while its feet stay planted, either as a canned cycle or interactively from a topic.

**Prerequisites:** the Go2 Gazebo demo works (see
[Demos → WBC on the real Unitree Go2](../reference/demos.md#wbc-on-the-real-unitree-go2-go2_gz)),
Gazebo Fortress installed, the workspace built.

## The idea in one paragraph

The whole-body controller already drives the robot toward a target posture every tick, in a
frame-consistent way. Standing just hands it *one fixed* target. Commanded motion hands it a
target that **changes over time** — a moving base pose — through the framework's `Tracking`
dialect. The controller has exactly the 6 residual degrees of freedom (with the feet
planted) to move the trunk, so the body follows the command while the feet stay put. Nothing
about the controller changes; only the target does.

## Step 1 — run the canned cycle

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py controllers:=go2_posture_controllers.yaml
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""   # release the startup weld
```

The Go2 sweeps through a squat + sway + tilt + yaw cycle (add `gui:=true` to watch). The
automated check asserts all four motions execute and the base stays stable:

```bash
bash kontrolem_bringup/test/e2e_go2_posture.sh
```

## Step 2 — command a pose interactively (live topic)

Launch the live config, then publish a base **offset** as a `geometry_msgs/Twist`
(`linear` = translation in metres, `angular` = roll/pitch/yaw in radians, both relative to
the nominal standing pose):

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py controllers:=go2_posture_live_controllers.yaml
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""

# squat down 1.5 cm and pitch the trunk nose-down ~5 deg:
ros2 topic pub --once /kontrolem_controller/base_target geometry_msgs/msg/Twist \
  "{linear: {z: -0.015}, angular: {y: 0.09}}"

# ...and back to nominal:
ros2 topic pub --once /kontrolem_controller/base_target geometry_msgs/msg/Twist "{}"
```

The body eases to the commanded pose and holds it; a zero command returns it to nominal.

## Step 3 — do it on the estimate (sim-to-real)

Add `base_source:=estimate` to command postures while the controller runs on the robot's
own [state estimate](estimate-the-base-state.md) — no ground truth in the loop:

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_posture_controllers.yaml base_source:=estimate
# automated:
bash kontrolem_bringup/test/e2e_go2_posture_closed.sh
```

The true base executes the postures while control sees only the estimate, which keeps
tracking the true base to ~2 mm / 0.02° through the motion.

## Tuning (`go2_posture_controllers.yaml`)

The canned motion is six per-axis harmonics `amp·cos(omega·t + phase)`, axes
`[x, y, z, roll, pitch, yaw]`:

```yaml
reference_type: base_pose
reference.base.amp:   [0.010, 0.012, 0.015, 0.06, 0.05, 0.08]   # m, m, m, rad, rad, rad
reference.base.omega: [0.25,  0.20,  0.25,  0.30, 0.35, 0.25]   # rad/s (slow = better tracking)
reference.base.phase: [2.5,   0.5,   0.0,   1.0,  1.5,  2.0]
```

!!! note "Two things that matter for base motion"
    **Relax the posture task.** `wbc.kp_post: 0` makes the joint-posture task a pure *damping*
    regularizer. With `kp_post > 0` it pulls the legs back to the standing shape and *fights*
    the articulation a base move needs (a squat must bend the knees) — the body then reaches
    only part of a commanded squat. Standing (`go2_stand_controllers.yaml`) keeps `kp_post: 25`.

    **Keep it slow, especially the squat.** Rotations (tilt/yaw) track tightly, but base
    *height* is bandwidth-limited — moving it pumps the whole body's weight against gravity
    through the feet. Command modest, slow squats; the rotations are the visually prominent,
    well-tracked motions.

## What this is (and isn't)

This is **commanded posture over planted feet** — the residual base DoF of a standing robot.
It is the stepping stone to **locomotion** (walking = base motion + swinging legs + feet
handing off contact), which is a separate, larger milestone: here the feet never leave the
ground.
