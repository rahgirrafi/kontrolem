# M0 end-to-end: QP task-space control regulating the 2-DoF fully-actuated arm
# through ros2_control, no Gazebo. Same runtime/pipeline as the cart-pole LQR
# demo — only the URDF (fully actuated) and the control law (qp) differ, which is
# the whole point of the v2 contract.
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="arm2.ros2_control.urdf",
        controllers_yaml="arm2_controllers.yaml",
    )
