# Automatic recovery (round-trip) demo: LQR balances the cart-pole until a scheduled
# shove trips it; the Supervisor fails over to MPC on its own, MPC rides out the
# disturbance, and then — once LQR is trustworthy again — the Supervisor hands BACK
# to LQR, all with no human command. Watch the round trip:
#     ros2 topic echo /kontrolem_controller/diagnostics   # control_law: lqr -> mpc -> lqr
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_pole_disturb.ros2_control.urdf",
        controllers_yaml="cart_pole_autorecover_controllers.yaml",
    )
