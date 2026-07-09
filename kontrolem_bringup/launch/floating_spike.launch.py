# M3 spike: load the FloatingBaseSimSystem, spawn the FloatingStateProbe, and
# watch the base fall + the probe reassemble the SE(3) state from scalar
# interfaces. Tests non-joint state through ros2_control end-to-end.
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from _common import _cmeel_lib_dirs  # noqa: E402
from launch.actions import SetEnvironmentVariable  # noqa: E402


def generate_launch_description():
    desc = get_package_share_directory("kontrolem_description")
    bringup = get_package_share_directory("kontrolem_bringup")
    with open(os.path.join(desc, "urdf", "floating_biped.ros2_control.urdf")) as f:
        urdf = f.read()
    rd = {"robot_description": urdf}
    controllers = os.path.join(bringup, "config", "floating_spike_controllers.yaml")

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[rd])
    cm = Node(package="controller_manager", executable="ros2_control_node", output="screen",
              parameters=[controllers, rd,
                          {"floating_state_probe.robot_description": urdf,
                           "floating_state_probe.joints": ["hip_left", "hip_right"]}])
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    probe = Node(package="controller_manager", executable="spawner",
                 arguments=["floating_state_probe", "-c", "/controller_manager"])
    delay = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[probe]))

    ld = ":".join(_cmeel_lib_dirs() + [os.environ.get("LD_LIBRARY_PATH", "")])
    return LaunchDescription([
        SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=ld), rsp, cm, jsb, delay])
