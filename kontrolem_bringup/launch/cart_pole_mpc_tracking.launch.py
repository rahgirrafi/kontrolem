# M2: tracking-MPC — MPC following a moving reference on the cart-pole through
# ros2_control (predictive tracking; uses the future reference over the horizon).
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_pole.ros2_control.urdf",
        controllers_yaml="cart_pole_mpc_tracking_controllers.yaml",
    )
