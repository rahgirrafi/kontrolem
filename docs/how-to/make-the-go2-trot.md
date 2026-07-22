# Make the Go2 trot (model-free)

**Goal:** walk the Go2 forward with a **dynamic diagonal trot** driven by the *model-free*
`KinematicGaitController` — no dynamics model, no QP, no state estimator.

**Prerequisites:** the Go2 Gazebo demo works (see
[Demos → WBC on the real Unitree Go2](../reference/demos.md#wbc-on-the-real-unitree-go2-go2_gz)),
Gazebo Fortress installed, the workspace built.

## The idea in one paragraph

This is the simplest way the framework can make a robot walk, and the first of the
["Go2 walks four ways" showcase](make-the-go2-walk.md#the-go2-walks-four-ways). Two diagonal
pairs of feet — {FL,RR} then {FR,RL} — swing 180° out of phase, so the two feet on the ground
always form a line under the body that catches it. Each tick the controller samples the gait
(where each foot should be), solves a small per-leg **inverse kinematics** for the joint angles
that put the foot there, and holds those angles with a **joint PD** on the effort interface.
There is no dynamics model, no optimization, and no estimator: the IK uses the *scheduled*
(nominal, forward-advancing) base rather than a measured one, so the controller is fully
**open-loop in the base**. This is the CHAMP lineage expressed as a `Controller` plugin behind
the same `compute()` contract as LQR/MPC/WBC.

## Step 1 — trot it

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_trot_kinematic.yaml estimator:=false base_source:=ecm
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""   # release the startup weld
```

Add `gui:=true` to watch. The Go2 holds nominal briefly (the gait waits out the startup weld),
then trots forward. Because a trot has no static weight-shift, it does **not** sway side to side
like the crawl. The automated check asserts it advances forward and stays upright:

```bash
bash kontrolem_bringup/test/e2e_go2_trot_kinematic.sh
```

## Tuning (`go2_trot_kinematic.yaml`)

```yaml
control_law: kinematic_gait
reference_type: trot
gait.swing_pair: [0, 1, 1, 0]   # diagonals: {FL,RR}=0, {FR,RL}=1 (contact-frame order FL,FR,RL,RR)
gait.period: 0.8                # one full stride (both pairs swing once); shorter = livelier
gait.duty: 0.5                  # swing fraction of a pair's half-cycle; <1 leaves a double-support margin
gait.step_len: 0.06             # forward step per foot per stride (m)
gait.step_h: 0.06               # swing-arc apex height (m)
gait.start_delay: 6.0           # hold nominal until well after the weld releases
kin.kp: 150.0                   # joint stiffness — must hold the stance legs (there is NO gravity comp)
kin.kd: 4.0                     # joint damping
kin.tau_max: 23.7               # per-joint torque clamp
kin.ik_max_iter: 20             # per-leg Gauss-Newton cap (hard-RT bound)
kin.ik_step_clamp: 0.5          # max Δq per IK iteration (rad) — singularity guard
```

!!! note "Stiff gains, on purpose"
    The controller is model-free, so there is **no gravity compensation** — the joint PD alone
    must hold the robot up. `kin.kp` therefore has to be fairly stiff (the demo holds the Go2
    with only ~1 cm of sag at `kp = 150`). Too soft and it sags; too stiff and it chatters.

!!! warning "Open-loop: it walks forward and upright, but drifts in heading"
    With no base or yaw feedback, the trot holds attitude and height beautifully but slowly
    **veers** off a straight line (~0.65 m of lateral drift over 30 s in the demo). That is the
    honest limitation of a model-free open-loop walk — and precisely the gap the closed-loop
    whole-body trot (which regulates the base against the [state estimate](estimate-the-base-state.md))
    exists to close. The e2e therefore asserts forward + upright, not straightness.

## What this is (and isn't)

This is the **model-free** corner of the showcase: minimal, robust, and estimator-free, but with
no force awareness and open-loop heading. The **same robot, same trot** run through the
whole-body controller (`control_law: wbc`) adds contact forces, friction limits, and closed-loop
base regulation — one `control_law:` line apart. See
[Make the Go2 trot (whole-body)](make-the-go2-trot-whole-body.md) for the same trot run that way.
