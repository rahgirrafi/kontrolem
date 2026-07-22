# Make the Go2 trot (whole-body)

**Goal:** walk the Go2 forward with the **same diagonal trot** as the
[model-free version](make-the-go2-trot.md), but driven by the **whole-body QP**
(`control_law: wbc`) — contact forces, friction cones, torque limits, full floating-base
dynamics, and **closed-loop** base regulation.

**Prerequisites:** the Go2 Gazebo demo works (see
[Demos → WBC on the real Unitree Go2](../reference/demos.md#wbc-on-the-real-unitree-go2-go2_gz)),
Gazebo Fortress installed, the workspace built.

## The idea in one paragraph

This is the *same robot, same trot* as the model-free demo — one `control_law:` line apart. Where
the [kinematic trot](make-the-go2-trot.md) samples the gait, solves a per-leg IK, and holds the
joints with a PD (open-loop in the base), the whole-body controller solves an **inverse-dynamics
QP** each tick: the two grounded diagonal feet are frictional contacts, the two flight feet track
their swing arc as tasks, and a stiff **base task** holds the trunk upright and glides it forward
— all subject to the friction pyramid and torque limits. Because the base task regulates against
the *measured* base (ground truth, or the [state estimate](estimate-the-base-state.md)), it is
**closed-loop**: it corrects heading and attitude instead of drifting. Behaviourally this is the
[static crawl walk](make-the-go2-walk.md) with the gait swapped from a crawl to a trot (and the
static weight-shift deleted) — the exact same `WbcController`, which accepts the `Locomotion`
dialect unchanged.

## Step 1 — trot it (on ground truth)

```bash
ros2 launch kontrolem_bringup go2_gz.launch.py \
  controllers:=go2_trot_wbc.yaml estimator:=false base_source:=ecm
ign topic -t /go2/detach -m ignition.msgs.Empty -p ""   # release the startup weld
```

Add `gui:=true` to watch. The Go2 holds nominal until the startup weld releases, then trots
forward on alternating diagonals. The automated check asserts forward + upright:

```bash
bash kontrolem_bringup/test/e2e_go2_trot_wbc.sh
```

## Step 2 — trot on the estimate (closed-loop sim2real)

Swap the base source to the robot's own [InEKF estimate](estimate-the-base-state.md) — the
controller now never sees the true base:

```bash
BASE=estimate EST=inekf bash kontrolem_bringup/test/e2e_go2_trot_wbc.sh
```

This mode **asserts** (forward + upright + a bounded estimate): the Go2 trots forward and
upright driven entirely by its own estimate, reliably (5/5 runs).

!!! tip "What actually made the trot reliable: the startup settle-gate, not the estimator"
    M14 shipped this mode as a report-only *frontier* — it tipped. Chasing that revealed the real
    cause, and it was **not** the estimator: the whole-body trot tipped **~2 of 3 runs even on
    perfect ground-truth state**, always at startup (a sideways roll off the two-diagonal support
    line). The gait was beginning to step on a fixed wall-clock (`start_delay`) that isn't
    synchronized with the Gazebo weld-release, so the first diagonal swing often fired while the
    base was still in the release transient — the same startup-timing race M12 found for the crawl.
    The fix is the **settle-gate** (`gait.settle_gate: true`): hold nominal all-stance until the
    base is *measured* to have settled after the release, then start stepping. With it, both ground
    truth and the estimate go **5/5**. (An offline estimator-in-the-loop harness had already shown
    the InEKF itself keeps the trot upright under IMU noise, bias, and contact flicker — the filter
    was never the bottleneck.)

!!! note "Residual: the estimate's absolute position drifts (~0.19 m), but does not tip"
    On the estimate the robot walks upright and forward, but the InEKF's *absolute* position
    tracks the truth to only ~0.19 m over 30 s (a slow, bounded, consistent drift — not a tip).
    That tight-tracking refinement (`< 0.10 m`) is reported by the e2e but not gated; it is a
    much smaller, non-destabilizing estimator matter, cleanly separated from the balance problem
    the settle-gate solved.

## Tuning (`go2_trot_wbc.yaml`)

```yaml
control_law: wbc
reference_type: trot
gait.swing_pair: [0, 1, 1, 0]   # diagonals: {FL,RR}=0, {FR,RL}=1 (contact-frame order FL,FR,RL,RR)
gait.period: 1.0                # one full stride (both pairs swing once)
gait.duty: 0.5                  # swing fraction of a pair's half-cycle; <1 keeps an all-stance bracket
gait.step_len: 0.06             # forward step per foot per stride (m)
gait.step_h: 0.05               # swing-arc apex height (m)
gait.start_delay: 0.5           # a short all-stance beat AFTER the settle-gate releases
gait.settle_gate: true          # don't step until the base settles post-weld-release (see tip above)
gait.settle_speed: 0.04         # m/s: base speed below which "settled"
gait.settle_tilt: 0.12          # rad: base tilt below which "settled"
wbc.kp_base: 100.0              # base task stiffness — holds the trunk upright over two diagonal feet
wbc.kd_base: 20.0
wbc.kp_post: 0.0                # posture = damping only, so a swing leg is free to follow its arc
wbc.kp_swing: 400.0             # swing-foot tracking stiffness
wbc.kd_swing: 40.0
wbc.mu: 0.5                     # friction coefficient (linearized pyramid)
wbc.tau_max: 23.7              # per-joint torque limit (enforced inside the QP)
```

!!! note "Quasi-static first"
    `duty: 0.5` leaves an all-four-stance bracket around each diagonal's swing, so there is never a
    flight phase (always ≥2 feet down). That keeps the trot quasi-static and forgiving — a
    whole-body controller holding the base still on just two diagonal feet is inherently marginal.
    Push toward a livelier trot (`duty`→1, shorter `period`) only once the quasi-static one is
    solid. The offline gate `test_wbc_trot` checks feasibility on two diagonal feet before Gazebo.

!!! tip "Closed-loop keeps it straight"
    Unlike the open-loop model-free trot (which veers ~0.65 m sideways over 30 s with no base
    feedback), the whole-body trot **regulates the base**, so it holds a near-straight line. That
    straightness — and force/friction awareness — is what the extra machinery (dynamics model, QP,
    estimator) buys over the model-free version.

## What this is (and isn't)

This is the **whole-body** corner of the ["Go2 walks four ways" showcase](make-the-go2-walk.md#the-go2-walks-four-ways):
the same gait as the model-free trot, but force-aware and closed-loop. It is still a *quasi-static*
trot (always ≥2 feet down); a truly **dynamic** trot with a flight phase — balanced by momentum
and footstep placement rather than a standing support — is a later milestone (it needs a
capture-point / MPC planning layer over this same WBC).
