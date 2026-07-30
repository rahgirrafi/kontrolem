# M17: gain-scheduling / LPV regulating the 2-DoF arm through ros2_control.
#
# Identical runtime, robot and pipeline to arm2.launch.py (the QP demo) — only the
# control_law and its parameters differ. LPV reached the runtime purely as a
# ControllerFactory plugin (M16's registry); no framework source changed to host it.
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="arm2.ros2_control.urdf",
        controllers_yaml="arm2_lpv_controllers.yaml",
    )
