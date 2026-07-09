# Multi-controller Supervisor demo: ONE runtime hosts LQR + MPC on the cart-pole
# and switches between them on a human command, bumplessly. Once running:
#     ros2 topic pub -1 /kontrolem_controller/switch_controller \
#         std_msgs/msg/String "{data: mpc}"
#     ros2 topic echo /kontrolem_controller/diagnostics    # control_law field flips
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_pole.ros2_control.urdf",
        controllers_yaml="cart_pole_switch_controllers.yaml",
    )
