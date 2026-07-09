# M4 end-to-end: a QP whole-body controller stands a floating quadruped through
# ros2_control, no Gazebo. Same shared pipeline as every other demo
# (_common.build_sim_launch) — the quad's contact-constrained sim + the WBC law
# are selected entirely by the URDF + controllers YAML.
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="floating_quadruped.ros2_control.urdf",
        controllers_yaml="quad_stand_controllers.yaml",
    )
