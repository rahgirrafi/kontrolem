# Per-tick telemetry from the Kontrol'Em runtime (KontrolemController). Named,
# self-describing fields — the observability surface for any hosted control law
# (LQR / LQG / QP / MPC), so behaviour is inspectable via `ros2 topic echo` and
# rosbag rather than a Float64MultiArray layout hack.
std_msgs/Header header

string control_law        # which law produced this (lqr | lqg | qp | mpc)

# State the controller acted on this tick (model joint order; for a floating
# base, q carries the SE(3) root so q.size() == nq, v.size() == nv).
float64[] q               # configuration (measured or estimated)
float64[] v               # velocity (0-filled for an output-feedback law)

float64[] q_ref           # reference configuration (setpoint or sampled trajectory)

float64[] tau             # command applied to the actuated joints this tick

# Supervisor / trust.
bool ok                   # control law's status().ok
float64 margin            # status().margin (distance to the trust boundary)
bool safe_action          # true if the supervisor overrode with the safe action

float64 update_us         # wall-clock duration of update() this tick, microseconds
int32 solver_iters        # QP iterations this tick (0 for closed-form laws); RT margin vs the max_iter cap
