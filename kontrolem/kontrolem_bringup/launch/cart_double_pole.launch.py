# LQR balancing the cart-DOUBLE-inverted-pendulum through ros2_control — the
# framework on a genuinely hard underactuated benchmark (1 actuator, 2 poles).
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_double_pole.ros2_control.urdf",
        controllers_yaml="cart_double_pole_controllers.yaml",
    )
