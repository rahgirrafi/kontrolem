# M1: LQG output-feedback compensator balancing the cart-pole through
# ros2_control. Same robot + runtime as cart_pole.launch.py — only the control
# law differs (lqg vs lqr), and LQG estimates velocity from positions alone.
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_pole.ros2_control.urdf",
        controllers_yaml="cart_pole_lqg_controllers.yaml",
    )
