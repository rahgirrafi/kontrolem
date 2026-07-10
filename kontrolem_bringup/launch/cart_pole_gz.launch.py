# M6.1 — independent-physics validation. The SAME KontrolemController (LQR) that
# balances the cart-pole against CartPoleSimSystem, now balancing it against Gazebo
# Fortress's own physics engine. The controller binary and cart_pole_controllers.yaml
# are UNCHANGED; only the plant differs (ign_ros2_control/IgnitionSystem instead of
# CartPoleSimSystem). That zero-controller-change swap is the whole point.
#
# Headless (server-only); watch it with:  ros2 topic echo /kontrolem_controller/diagnostics
import os
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


sys.path.insert(0, os.path.dirname(__file__))
from _common import _cmeel_lib_dirs  # noqa: E402


def generate_launch_description():
    description_pkg = get_package_share_directory("kontrolem_description")
    bringup_pkg = get_package_share_directory("kontrolem_bringup")

    world_path = os.path.join(description_pkg, "worlds", "cart_pole.world.sdf")
    ctrl_yaml = os.path.join(bringup_pkg, "config", "cart_pole_controllers.yaml")
    gz_urdf_template = os.path.join(description_pkg, "urdf", "cart_pole_gz.urdf")

    gui_arg = DeclareLaunchArgument("gui", default_value="false", description="Launch Gazebo GUI",
)
    # The controller parses this (clean) URDF with Pinocchio; it ignores the
    # ros2_control tags and never sees the gz world-anchor.
    clean_urdf_path = os.path.join(description_pkg, "urdf", "cart_pole.ros2_control.urdf")
    with open(clean_urdf_path) as f:
        clean_urdf = f.read()

    tmp = tempfile.mkdtemp(prefix="kontrolem_gz_")

    # robot_description param for the controller (Humble's base can't fetch it).
    desc_yaml = os.path.join(tmp, "robot_description.yaml")
    with open(desc_yaml, "w") as f:
        yaml.safe_dump(
            {"kontrolem_controller": {"ros__parameters": {"robot_description": clean_urdf}}},
            f, default_flow_style=False)

    # Substitute the two <parameters> paths into the gz URDF, write the spawn file.
    with open(gz_urdf_template) as f:
        gz_urdf = f.read().replace("__CTRL_YAML__", ctrl_yaml).replace("__DESC_YAML__", desc_yaml)
    gz_urdf_path = os.path.join(tmp, "cart_pole_gz.urdf")
    with open(gz_urdf_path, "w") as f:
        f.write(gz_urdf)

    # cmeel Pinocchio libs must be visible to the gz process (it dlopens our
    # controller plugin) and the spawners; the ign_ros2_control system plugin lives
    # in the ROS lib dir, so gz must search there too.
    ros_lib = "/opt/ros/humble/lib"
    ld = ":".join(_cmeel_lib_dirs() + [ros_lib, os.environ.get("LD_LIBRARY_PATH", "")])
    set_ld = SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=ld)
    plugin_path = ":".join(
        [ros_lib, os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")])
    set_plugin_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH", value=plugin_path)

    # Run immediately (-r). The pole starts upright and stays there (no perturbation)
    # through the seconds before LQR activates; the test perturbs it only afterwards.
    gazebo_headless = ExecuteProcess(
    condition=UnlessCondition(LaunchConfiguration("gui")),
    cmd=[
        "ign",
        "gazebo",
        "-s",
        "-r",
        "-v",
        "3",
        world_path,
    ],
    output="screen",
)
    gazebo_gui = ExecuteProcess(
    condition=IfCondition(LaunchConfiguration("gui")),
    cmd=[
        "ign",
        "gazebo",
        "-r",
        "-v",
        "3",
        world_path,
    ],
    output="screen",
)

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-world", "cart_pole", "-file", gz_urdf_path, "-name", "cart_pole"],
        output="screen",
    )

    # RSP must publish the GZ URDF: gz_ros2_control reads /robot_description from here
    # to discover the ros2_control hardware (ign_ros2_control/IgnitionSystem) + joints.
    # (The controller reads the CLEAN URDF separately via its own robot_description
    # param, for Pinocchio — RSP ignores the <gazebo>/<ros2_control> tags.)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": gz_urdf}],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kontrolem_controller", "--controller-manager", "/controller_manager"],
    )
    # Load controllers only after the model (and its embedded controller_manager) is up.
    after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn, on_exit=[jsb_spawner]))
    after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jsb_spawner, on_exit=[ctrl_spawner]))

    return LaunchDescription([
        set_ld, 
        set_plugin_path,
        gazebo_headless,
        gazebo_gui,
        robot_state_publisher, 
        spawn,
        after_spawn, 
        after_jsb,
    ])
