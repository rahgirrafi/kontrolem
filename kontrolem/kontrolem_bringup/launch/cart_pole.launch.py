# M0 end-to-end: LQR balancing the cart-pole through ros2_control, no Gazebo.
# See _common.build_sim_launch for the shared pipeline
# (robot_state_publisher + ros2_control_node + joint_state_broadcaster + law).
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="cart_pole.ros2_control.urdf",
        controllers_yaml="cart_pole_controllers.yaml",
    )
