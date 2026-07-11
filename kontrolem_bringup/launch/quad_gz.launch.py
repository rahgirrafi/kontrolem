# M6.2 — WBC under Gazebo's real contact/friction. The SAME KontrolemController
# (control_law: wbc) + quad_stand_controllers.yaml that stand the quadruped against
# FloatingContactSimSystem (pinned feet), now standing it against Gazebo's own
# contact solver: joints via ign_ros2_control/IgnitionSystem, base pose/twist +
# contact via kontrolem_gz/GzBaseStateSystem (read from the ECM), controller_manager
# booted in-gz. Only the plant differs — that zero-controller-change swap is the point.
#
# Headless (server-only). Watch:  ros2 topic echo /kontrolem_controller/diagnostics
import os
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from _common import _cmeel_lib_dirs  # noqa: E402


def generate_launch_description():
    desc = get_package_share_directory("kontrolem_description")
    bringup = get_package_share_directory("kontrolem_bringup")

    world_path = os.path.join(desc, "worlds", "quadruped.world.sdf")
    ctrl_yaml = os.path.join(bringup, "config", "quad_stand_controllers.yaml")
    gz_urdf_template = os.path.join(desc, "urdf", "floating_quadruped_gz.urdf")
    base_height = "0.2753"

    gui_arg = DeclareLaunchArgument("gui", default_value="false")

    # Clean URDF the controller parses with Pinocchio (floating base); it ignores
    # the ros2_control/gazebo tags. Same body as the gz URDF.
    clean_urdf_path = os.path.join(desc, "urdf", "floating_quadruped.ros2_control.urdf")
    with open(clean_urdf_path) as f:
        clean_urdf = f.read()

    tmp = tempfile.mkdtemp(prefix="kontrolem_quad_gz_")
    desc_yaml = os.path.join(tmp, "robot_description.yaml")
    with open(desc_yaml, "w") as f:
        yaml.safe_dump(
            {"kontrolem_controller": {"ros__parameters": {"robot_description": clean_urdf}}},
            f, default_flow_style=False)

    with open(gz_urdf_template) as f:
        gz_urdf = f.read().replace("__CTRL_YAML__", ctrl_yaml).replace("__DESC_YAML__", desc_yaml)
    gz_urdf_path = os.path.join(tmp, "floating_quadruped_gz.urdf")
    with open(gz_urdf_path, "w") as f:
        f.write(gz_urdf)

    ros_lib = "/opt/ros/humble/lib"
    ld = ":".join(_cmeel_lib_dirs() + [ros_lib, os.environ.get("LD_LIBRARY_PATH", "")])
    set_ld = SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=ld)
    set_plugin = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=":".join([ros_lib, os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")]))

    gazebo_headless = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration("gui")),
        cmd=["ign", "gazebo", "-s", "-r", "-v", "3", world_path], output="screen")
    gazebo_gui = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("gui")),
        cmd=["ign", "gazebo", "-r", "-v", "3", world_path], output="screen")

    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-world", "quadruped", "-file", gz_urdf_path,
                   "-name", "floating_quadruped", "-z", base_height],
        output="screen")

    # RSP publishes the GZ URDF so gz_ros2_control discovers the hardware + joints.
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[{"robot_description": gz_urdf}])

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    wbc = Node(package="controller_manager", executable="spawner",
               arguments=["kontrolem_controller", "-c", "/controller_manager"])
    # Activate the WBC as fast as possible (in parallel with jsb, not chained after
    # it): the quadruped collapses under gravity during the spawn->activation gap,
    # so every saved second means less sag for the WBC to recover from.
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, wbc]))

    return LaunchDescription([
        gui_arg, set_ld, set_plugin,
        gazebo_headless, gazebo_gui, rsp, spawn, after_spawn])
