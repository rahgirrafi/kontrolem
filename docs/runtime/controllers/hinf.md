# hinf_controller

Output-feedback controller for an H∞ artifact (`hinf` or `hinf_mixsyn`).

**Law.** H∞ synthesis produces a dynamic controller structurally identical to
LQG — a compensator `ctrl_A/ctrl_B/ctrl_C/ctrl_D` driven by the measured output
deviation — so it runs on the same `OutputFeedbackController` base and the same
Tustin discretization. The package only widens the accepted `controller_type`
to `{hinf, hinf_mixsyn}`.

**Measurements.** The demo artifact was synthesized with position *and* velocity
outputs (`cart_joint.q`, `joint1.q`, `cart_joint.qd`, `joint1.qd`), so the
controller claims velocity state too and the validity guard enforces both
position and velocity bounds. Whether velocities are used is read from the
artifact's `output_names`, not fixed in the plugin.

**Synthesis note (design-time).** On this plant the γ-optimal central controller
carries ultra-fast modes (~1e9 rad/s) that no 100 Hz loop can realize; the
exported artifact comes from a **suboptimal** design at a backed-off γ, whose
discrete closed loop is verifiably stable. This is a design-side concern —
documented here because it explains why the H∞ artifact's matrices are modest in
magnitude and the controller is deployable. See the parent repo's H∞ design docs.

**Discretization rate.** As with LQG, set `update_rate` equal to
`controller_manager.update_rate`.

## Demo

```bash
ros2 launch hinf_controller hinf_demo.launch.py
```
