# How to run the Gazebo quadruped and push it

> **For:** anyone who wants to see the whole-body controller (WBC) hold a stance and reject a real shove under Gazebo's own physics. **Assumes:** a built, sourced workspace with **Gazebo Fortress** installed. **Goal:** launch `quad_gz`, hand the robot to the WBC, apply a push, and watch it recover — in the correct order, with the traps called out.

This is the Gazebo variant of the standing demo. Unlike the pinned-feet sim ([Tutorial 2](../tutorials/stand-a-quadruped.md)), here the feet are **real unilateral contacts** on a friction surface — the robot only stays up because the WBC is actively balancing it. That realism comes with a startup dance and a strict command order. Follow the steps and it just works; skip a step and you get the classic "nothing happens."

## The mental model: weld → activate → detach → push

Gazebo has no "hold the robot still until the controller is ready" feature. So the demo uses one:

1. **Weld.** At spawn the base is welded to a static anchor (a `DetachableJoint`). A DART quirk freezes the *entire* model rigid at the exact nominal standing posture. The robot physically cannot fall — but it also cannot be controlled or pushed. It's a statue.
2. **Activate.** Meanwhile the controller_manager loads and the WBC (`kontrolem_controller`) configures and activates. Until it reports **active**, it is not sending torques.
3. **Detach.** Once the WBC is active, you publish the detach command. The weld dissolves and the robot becomes a free body held up **only** by the controller balancing it against real gravity and contact. *This is the moment the demo becomes real.*
4. **Push.** Now — and only now — an external force actually perturbs the robot, and you get to watch the WBC catch it.

```
spawn ──[welded: frozen upright, uncontrollable]──► WBC active ──[detach]──► WBC balances for real ──► push
```

!!! warning "The order is the whole game"
    A push **before** detach does nothing — the weld absorbs all of it (this is the #1 cause of "nothing happens"). A detach **before** the WBC is active drops the robot on the floor, because nothing is holding it up. Always: **launch → verify active → detach → push.**

## Step 1 — Launch, headless and sourced

Launch from a shell that has sourced the workspace. Headless (the default) is the validated path:

```bash
cd <workspace>
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch kontrolem_bringup quad_gz.launch.py        # gui defaults to false
```

Leave it running. In the launch log you should see the base-state hardware bind and the WBC activate:

```
[GzBaseStateSystem]: gz base-state bound: model 'floating_quadruped' link 'base_link'
[resource_manager]: Successful 'activate' of hardware 'base_state'
[spawner_kontrolem_controller]: Configured and activated kontrolem_controller
```

If you don't see those three lines, do **not** proceed — see [Troubleshooting](#troubleshooting-nothing-happens) below.

!!! tip "Same recipe for the real Unitree Go2 (`go2_gz`)"
    Everything on this page applies unchanged to the real Go2 demo — only the names differ. Launch `go2_gz.launch.py` instead, and substitute throughout: model `floating_quadruped` → **`go2`**, base link `base_link` → **`base`**, detach topic `/quadruped/detach` → **`/go2/detach`**, wrench topic `/world/quadruped/wrench` → **`/world/go2/wrench`**, and the push entity `floating_quadruped::base_link` → **`go2::base`**. Go2 is ~1.6× heavier, so use a firmer push (~**5000 N** for a ~6 cm shove vs the toy's 3000 N). See [Reference → Demos](../reference/demos.md#wbc-on-the-real-unitree-go2-go2_gz).

## Step 2 — Verify the WBC is actually active

In a **second sourced terminal**, confirm the controller is live and its floating-base interfaces exist:

```bash
ros2 control list_controllers                              # kontrolem_controller ... active
ros2 control list_hardware_interfaces | grep floating_base # expect 13 interfaces
```

You must see `kontrolem_controller … active` **and** the 13 `floating_base/*` interfaces. If the controller is `inactive`, or those interfaces are missing, the base-state hardware didn't load — the robot is standing only because it's still welded, and a push will do nothing.

## Step 3 — Detach: hand the robot to the controller

```bash
ign topic -t /quadruped/detach -m ignition.msgs.Empty -p ""
```

The robot is now free and standing purely on the WBC. If you're watching (`gui:=true`), you'll see it settle by a millimetre or two and hold. If it collapses here, the WBC was not actually active — go back to Step 2.

## Step 4 — Push and watch the recovery

Apply a one-shot lateral force to the base. `3000` N is the validated shove (~6 cm excursion, recovers to millimetres):

```bash
ign topic -t /world/quadruped/wrench -m ignition.msgs.EntityWrench \
  -p 'entity {name:"floating_quadruped::base_link" type:LINK} wrench {force {y: 3000}}'
```

Watch the WBC work while you push, in a third terminal:

```bash
ros2 topic echo /kontrolem_controller/diagnostics
```

`ok: true` should hold throughout; the base position deflects then returns to nominal; `margin` (friction headroom, N) stays positive.

### Choosing the force

| Force | Effect | Notes |
|---|---|---|
| `300`–`800` N | Barely visible | A one-shot wrench is a **2 ms impulse** (~0.4 N·s at 300 N). The WBC erases it almost instantly. |
| `3000` N | ~6 cm excursion, full recovery | The validated shove. Cone margin stays healthy. |
| `6000` N | Tips over | Correctly overwhelms it — the friction cone saturates and `status().ok` goes false. Success is bounded, not staged. |

!!! note "Want a sustained lean, not a tap?"
    A one-shot wrench lasts a single physics step, so small forces are invisible. For a *held* push you can actually see, use the persistent topic and clear it when done:
    ```bash
    ign topic -t /world/quadruped/wrench/persistent -m ignition.msgs.EntityWrench \
      -p 'entity {name:"floating_quadruped::base_link" type:LINK} wrench {force {y: 400}}'
    # ... watch the WBC lean into it ...
    ign topic -t /world/quadruped/wrench/clear -m ignition.msgs.Entity \
      -p 'name:"floating_quadruped::base_link" type:LINK'
    ```
    The `EntityWrench` message has **no duration field** — persistent-then-clear is the only way to hold a force.

## Step 5 — Or just run the automated test

The whole sequence (launch → detach → assert stance → push → assert recovery) is captured in one script:

```bash
bash kontrolem_bringup/test/e2e_quad_gz.sh
```

Expected tail:

```
PASS: Gazebo WBC — stance(...)=True push(...)=True recovery(...)=True
```

## Troubleshooting: "nothing happens" {#troubleshooting-nothing-happens}

Almost every failure is one of these. Work down the list:

| Symptom | Cause | Fix |
|---|---|---|
| Push does nothing, robot rock-solid | **Still welded** — you pushed before detaching, or the detach didn't fire | Confirm `/model/floating_quadruped/detachable_joint/state` behaviour, then publish `/quadruped/detach` (Step 3) |
| `list_controllers` shows `kontrolem_controller inactive` | Either you checked before boot finished, or `base_state` never loaded | Wait for the three log lines; if they never come, it's the shell (see below), not the GUI |
| Error `State interface 'floating_base/pose.position.x' does not exist` | `base_state` / `GzBaseStateSystem` didn't load, so its `floating_base` interfaces don't exist for the WBC to claim | **Stale or unsourced workspace** — `source install/setup.bash` in *this* shell after a fresh build. Verified: this is *not* caused by `gui:=true` (A/B tested). Not a controller bug |
| Robot collapses right after detach | WBC was not truly active when you detached | Detach only *after* Step 2 confirms `active` |
| `/world/quadruped/wrench` not in `ign topic -l` | **Not a fault.** That topic only appears once a publisher connects | Just publish to it (Step 4); it materialises on first publish |

!!! warning "`ign topic -l` can't see the controller"
    `ign topic -l` lists **Ignition transport** topics only — it says nothing about whether the ROS controller is active. Seeing `/gui/camera/pose` there tells you the **GUI** variant is running; it does *not* tell you the WBC is up. Always check the ROS side with `ros2 control list_controllers` (Step 2).

!!! note "Headless vs. GUI — both are equivalent"
    An A/B log diff (`gui:=false` vs `gui:=true`, same machine) confirms **the GUI variant loads the exact same hardware and activates the WBC identically** — both load `floating_quadruped_gz` + `base_state`, activate `base_state`, and activate `kontrolem_controller`, with zero "interface does not exist" errors and no render/crash failures (the GUI adds only benign Qt/QML deprecation warnings to the log). The control path is not even slower: spawner→activate was ~1.6 s with GUI vs ~1.8 s headless. So `gui:=true` is safe — use it whenever you want to watch. Headless (`gui:=false`) remains the default only because it's lighter and needs no display (better for automation/CI). If you ever saw the WBC "inactive" under the GUI, the cause was **the shell, not the GUI** — see the note below.

!!! warning "The real cause of a genuinely-inactive WBC: the shell"
    If `base_state` truly never loads (the three log lines never appear, and `list_controllers` stays `inactive` well after boot), the launch shell did not have the built workspace on its loader path — a **stale or unsourced `install/`**. Always, in the same terminal you launch from:
    ```bash
    source /opt/ros/humble/setup.bash && source install/setup.bash   # after any rebuild
    ```
    The `GzBaseStateSystem` plugin (`kontrolem_gz`) is discovered through the sourced overlay; without it, the gz controller_manager loads only the joint hardware, the `floating_base` interfaces never exist, and the WBC can't activate — exactly the error above.

## Why this is set up the way it is

- The weld/detach exists because Gazebo has no built-in hold; the pinned-feet sim ([`quad_stand`](../reference/demos.md)) auto-releases instead, but Gazebo needs an explicit anchor. See [Reference → Demos](../reference/demos.md#wbc-vs-real-contact-quad_gz).
- Two hard-won integration facts (from `DEVELOPMENT.md`, M6.2): **a welded model ignores all applied joint torque** — never diagnose the controller against a welded robot — and **high SDF joint damping makes DART swallow commanded joint forces**, so keep `<dynamics damping>` near zero and let the controller supply the damping.
- The deeper "how does a floating base ride through ros2_control" story is in [Explanation → State & non-joint data](../explanation/state-and-non-joint-data.md).
