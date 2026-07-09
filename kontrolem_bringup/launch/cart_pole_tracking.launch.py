# M1: LQR following a moving reference (Tracking dialect) on the cart-pole. Same
# robot + runtime + gain law as cart_pole.launch.py — only the PROBLEM differs
# (Tracking vs Regulation), which is the point: one controller, two dialects.
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_pole.ros2_control.urdf",
        controllers_yaml="cart_pole_tracking_controllers.yaml",
    )
