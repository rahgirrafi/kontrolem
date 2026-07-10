# Automatic fail-forward demo: LQR balances the cart-pole until a scheduled shove
# drives it out of LQR's validity region; the Supervisor then fails over to MPC on
# its OWN (no human command) and rides out the disturbance. Watch it happen:
#     ros2 topic echo /kontrolem_controller/diagnostics   # control_law flips lqr->mpc
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_pole_disturb.ros2_control.urdf",
        controllers_yaml="cart_pole_autofallback_controllers.yaml",
    )
