# M4 push-recovery: the same standing quadruped + WBC as quad_stand, but the sim
# delivers a scheduled external base push (a disturbance) a couple of seconds in.
# The whole-body controller must catch it and drive the base back to nominal.
# Identical controller config to quad_stand — only the URDF (with push params) differs.
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="floating_quadruped_push.ros2_control.urdf",
        controllers_yaml="quad_stand_controllers.yaml",
    )
