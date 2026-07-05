# lqg_controller

Output-feedback controller for an `lqg` artifact.

**Law.** The exported compensator is the Kalman observer and the LQR regulator
folded into a single LTI system driven by the measured output deviation `y`:

$$\dot{\xi} = \texttt{ctrl\_A}\,\xi + \texttt{ctrl\_B}\,y,\qquad
u = u_{eq} + \texttt{ctrl\_C}\,\xi + \texttt{ctrl\_D}\,y.$$

It is Tustin-discretized once in `on_configure()` at the controller rate and
stepped every cycle. This is why LQG needs no separate Kalman gain and no state
estimate in the artifact — it is all inside `ctrl_*`.

**Measurements.** The demo artifact uses position-only outputs
(`cart_joint.q`, `joint1.q`), so the controller reads positions only and
reconstructs the rest through the observer — the case where LQG beats bare LQR.
If an artifact declares velocity outputs (`<joint>.qd`), velocity state is
claimed automatically.

**Artifact.** Requires the complete `ctrl_A/ctrl_B/ctrl_C/ctrl_D` group (no
`K`). Loading a static-gain (`lqr`) artifact is rejected at configure time.

**Discretization rate.** Set `update_rate` on the controller equal to
`controller_manager.update_rate`. The manager only propagates its rate to a
controller *after* `on_configure()` — too late to discretize — so the explicit
parameter is required for the dynamic compensators.

## Demo

```bash
ros2 launch lqg_controller lqg_demo.launch.py
```
