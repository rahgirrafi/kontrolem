# Make the Go2 walk

**Goal:** walk the standing Go2 forward with a statically-stable crawl gait — one foot
swinging at a time, feet leaving and rejoining the ground.

**Prerequisites:** the Go2 Gazebo demo works (see
[Demos → WBC on the real Unitree Go2](../reference/demos.md#wbc-on-the-real-unitree-go2-go2_gz)),
Gazebo Fortress installed, the workspace built.

## The idea in one paragraph

Walking is the same whole-body controller with two additions. First, it can now handle a
foot **in the air**: a swinging foot stops pushing on the ground and instead follows a
lift-forward-place arc, driven by the leg's joints. Second, a **crawl gait** decides, at
each instant, which foot is swinging and where the body should be — and it keeps the body's
centre of mass over the triangle of the three planted feet so the robot is *statically
stable* (it would not tip even if it froze mid-step). One foot at a time steps forward, so
the whole robot ratchets forward.

## Step 1 — walk it

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py controllers:=go2_walk_controllers.yaml
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""   # release the startup weld
```

Add `gui:=true` to watch. The Go2 holds still briefly (the gait waits out the startup
weld), then starts walking forward, swaying side to side as it shifts its weight between
steps. The automated check asserts it advances forward, stays upright, and actually steps:

```bash
bash kontrolem_bringup/test/e2e_go2_walk.sh
```

## Tuning (`go2_walk_controllers.yaml`)

```yaml
reference_type: gait
gait.order: [2, 0, 3, 1]   # swing sequence of contact_frames indices (RL, FL, RR, FR)
gait.period: 14.0          # one full 4-foot cycle (s) — slower = safer, more stable
gait.duty: 0.5             # fraction of a foot's quarter-cycle spent swinging
gait.step_len: 0.04        # forward step per foot per cycle (m)
gait.step_h: 0.045         # swing-arc apex height (m)
gait.base_gain: 1.0        # how fully the base tracks the support-polygon centroid
gait.start_delay: 6.0      # hold nominal until well after the weld releases
wbc.kp_swing: 400.0        # swing-foot tracking stiffness
wbc.kd_swing: 40.0
wbc.kp_post: 0.0           # posture = damping only, so the swing leg is free (as postures)
```

!!! note "Keep it slow and deliberate"
    A static crawl is inherently slow — it must shift the whole body's weight over the
    support triangle before each step. Larger/faster steps shrink the stability margin.
    Start gentle.

!!! warning "Walking on the estimate is improved but not yet fully reliable"
    `base_source:=estimate` walks on the robot's own [state estimate](estimate-the-base-state.md)
    instead of ground truth. Standing and postures on the estimate are solid, but **walking**
    stresses leg odometry: when feet leave and rejoin the ground the sensed contact flickers,
    and a briefly-mis-sensed swing foot can corrupt it. The **InEKF** (`estimator_type:=inekf`,
    M11) is the covariance-weighted estimator that handles this — it walks forward & upright in
    most runs and tighter than the M8 complementary filter (which poisons a shared
    least-squares), but it **still occasionally diverges**. Full reliability needs the next
    layer: feeding the gait's **planned** contact schedule to the estimator (killing the sensed
    flicker at the source) and/or IMU-bias states. `wbc.kp_post: 0`, a `flat_ground` height
    pin, and a gentle gait all help. Probe it with:

    ```bash
    EST=inekf bash kontrolem_bringup/test/e2e_go2_walk_closed.sh          # M11 InEKF
    EST=complementary bash kontrolem_bringup/test/e2e_go2_walk_closed.sh  # M8 baseline
    ```

## What this is (and isn't)

This is a **statically-stable crawl** — always three feet down, the CoM inside their
triangle. It is the stepping stone to **dynamic** locomotion (trot, run, jump), where the
robot is balanced by momentum rather than a static support polygon. A model-free **dynamic
trot** already ships — see [Make the Go2 trot](make-the-go2-trot.md).

## The Go2 walks four ways

Walking is a **showcase** of the framework's range: the *same* Go2, in the *same* world, driven
by structurally different controllers behind the one `compute()` contract — change a single
`control_law:` line to switch paradigm.

| Controller | `control_law` | What it is | Config |
|---|---|---|---|
| Kinematic trot | `kinematic_gait` | Model-free: gait → IK → joint PD. No dynamics/estimator; open-loop. | `go2_trot_kinematic.yaml` — [how-to](make-the-go2-trot.md) |
| Whole-body crawl | `wbc` | Inverse-dynamics QP: contact forces, friction, torque limits; closed-loop base. | `go2_walk_controllers.yaml` (this page) |
| Whole-body trot | `wbc` | The same QP as the crawl, driven by the diagonal trot gait — force-aware, closed-loop, straighter than the model-free trot. | `go2_trot_wbc.yaml` — [how-to](make-the-go2-trot-whole-body.md) |
| Convex MPC + WBC | *(planned)* | An optimal-control planning layer over the WBC. | — |
| RL policy | *(planned)* | A learned MLP loaded via synthesize=train / configure=load / compute=infer. | — |

The model-free trot is the simplest and most robust in sim; the WBC adds force awareness and
closed-loop base regulation. Two more paradigms (MPC, RL) round out the showcase.
