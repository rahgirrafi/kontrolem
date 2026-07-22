# kontrolem_locomotion

Concrete **gait patterns** for Kontrol'Em v2 — morphology-specific `GaitSource`
implementations that emit the per-tick `GaitPlan` a whole-body or kinematic
controller executes to walk. Header-only, ROS-independent.

## Purpose

`kontrolem_control` defines the **seam** — `GaitSource` ("at time `t`, which feet
are down, where does the swinging foot go, where does the body go?") and the
`GaitPlan` it fills. This package holds the **implementations** of that seam:

- `CrawlGait` (`crawl_gait.hpp`) — a statically-stable quadruped crawl (one foot
  swings at a time; base tracks the support-triangle centroid). Used by M10.
- `TrotGait` (`trot_gait.hpp`) — a dynamic quadruped trot (diagonal pairs swing
  together; **no** support-centroid weight-shift; base held at nominal, advanced
  forward). Used by the M13 kinematic-gait and M14 whole-body-trot showcases.

Both are **morphology-specific** (they assume four feet / diagonal pairs), which
is exactly why they live here rather than in the core contract: robot-specific
patterns must not accrete in `kontrolem_control`. Humanoid / biped gaits will join
this package as they are built. The *controllers* that consume a gait stay
morphology-agnostic (they take any `GaitSource`), so only the **patterns** are
robot-specific.

## Dependencies and build instructions

Depends only on `kontrolem_control` (for the `GaitSource` / `GaitPlan` seam) and
Eigen. Header-only `INTERFACE` library — nothing to compile.

```bash
# from the workspace root
P="$HOME/.local/lib/python3.10/site-packages/cmeel.prefix"
colcon build --packages-select kontrolem_locomotion \
  --cmake-args -DCMAKE_PREFIX_PATH="$P" -DCMAKE_BUILD_TYPE=Release
```

## Relation to other packages

- **`kontrolem_control`** — defines the `GaitSource`/`GaitPlan` seam these gaits
  implement (contract ← impl, the same split as controller-contract ↔ controllers).
- **`kontrolem_controllers`** — its `WbcController` (and the upcoming
  `KinematicGaitController`) *consume* a `GaitSource` to walk a robot.
- **`kontrolem_ros2_control`** — the runtime builds a concrete gait from
  `reference_type: gait` params and hands it to the active controller.

## Position in the complete architecture

Layer 3 (impl), ROS-independent core. It is the locomotion analogue of
`kontrolem_controllers`: `kontrolem_control` is the contract, this package is one
family of implementations against it. No ROS, no model, no solver dependencies —
just closed-form, allocation-free reference generation.

## Intended use

Provide ready gait patterns a controller can execute, and a clear home for new
patterns (per morphology / gait style) without touching the core contract or the
controller implementations.

## How to use it

```cpp
#include "kontrolem_locomotion/trot_gait.hpp"

kontrolem_locomotion::TrotGait gait;
gait.q_nominal = q_nominal;          // base pose + held joint posture (nq)
gait.foot_nominal = feet_at_t0;      // 4 world foot positions (FK once)
gait.period = 1.0; gait.step_len = 0.06;

kontrolem_control::GaitPlan plan;
plan.resize(/*nfeet=*/4, model.nq(), model.nv());   // off the RT path
gait.sample(t, plan);                                // per tick, allocation-free
// plan.stance / plan.swing_pos / plan.q_ref -> feed the controller
```
