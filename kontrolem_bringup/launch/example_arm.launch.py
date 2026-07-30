# M16 acceptance: the 2-DoF arm regulated by an OUT-OF-TREE control law
# (kontrolem_controller_example), selected with `control_law: example`. Same
# runtime/URDF as arm2.launch.py — only the controllers YAML (a different,
# third-party law) differs, which is the whole point of the M16 registry.
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _common import build_sim_launch  # noqa: E402


def generate_launch_description():
    return build_sim_launch(
        urdf_file="arm2.ros2_control.urdf",
        controllers_yaml="example_controllers.yaml",
    )
