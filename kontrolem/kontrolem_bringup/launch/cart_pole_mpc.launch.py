# M2: linear MPC balancing the cart-pole through ros2_control. Same robot +
# runtime as the other demos — the control law (mpc) solves a constrained
# receding-horizon QP each tick and applies the first input.
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_pole.ros2_control.urdf",
        controllers_yaml="cart_pole_mpc_controllers.yaml",
    )
