# M7/M8 — the real Unitree Go2 held up by the WBC under Gazebo's real contact/friction.
# The SAME KontrolemController (control_law: wbc) + go2_stand_controllers.yaml pattern
# that stands the toy floating_quadruped, now standing the Go2: joints via
# ign_ros2_control/IgnitionSystem, base pose/twist + contact via
# kontrolem_gz/GzBaseStateSystem, controller_manager booted in-gz.
#
# M8 adds a floating-base state estimator (base_estimator) and a base_source switch:
#   base_source:=ecm       WBC stands on Gazebo GROUND TRUTH (M7 / open-loop validation)
#   base_source:=estimate  WBC stands on the estimator's /base_odom (closed-loop sim2real)
#
# Headless (server-only). Watch:  ros2 topic echo /kontrolem_controller/diagnostics
# Push:  ign topic -t /world/go2/wrench -m ignition.msgs.EntityWrench \
#          -p 'entity {name:"go2::base" type:LINK} wrench {force {y: 200}}'
import os
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from _common import _cmeel_lib_dirs  # noqa: E402


def launch_setup(context, *args, **kwargs):
    desc = get_package_share_directory("kontrolem_description")
    bringup = get_package_share_directory("kontrolem_bringup")

    world_path = os.path.join(desc, "worlds", "go2.world.sdf")
    ctrl_yaml = os.path.join(bringup, "config", "go2_stand_controllers.yaml")
    gz_urdf_template = os.path.join(desc, "urdf", "go2_gz.urdf")
    base_height = "0.2868"  # offline-proven (feet grounded); matches world anchor + yaml
    # base_source is baked into the generated URDF text, so resolve it now (perform).
    base_source = LaunchConfiguration("base_source").perform(context)

    # Clean URDF the controller parses with Pinocchio (floating base); it ignores
    # the ros2_control/gazebo tags. Same body as the gz URDF.
    clean_urdf_path = os.path.join(desc, "urdf", "go2.ros2_control.urdf")
    with open(clean_urdf_path) as f:
        clean_urdf = f.read()

    tmp = tempfile.mkdtemp(prefix="kontrolem_go2_gz_")
    desc_yaml = os.path.join(tmp, "robot_description.yaml")
    with open(desc_yaml, "w") as f:
        yaml.safe_dump(
            {"kontrolem_controller": {"ros__parameters": {"robot_description": clean_urdf}},
             # M8: the estimator parses the SAME clean floating URDF (FK/Jacobian).
             "base_estimator": {"ros__parameters": {"robot_description": clean_urdf}}},
            f, default_flow_style=False)

    with open(gz_urdf_template) as f:
        gz_urdf = (f.read().replace("__CTRL_YAML__", ctrl_yaml)
                   .replace("__DESC_YAML__", desc_yaml)
                   .replace("__BASE_SOURCE__", base_source))
    gz_urdf_path = os.path.join(tmp, "go2_gz.urdf")
    with open(gz_urdf_path, "w") as f:
        f.write(gz_urdf)

    ros_lib = "/opt/ros/humble/lib"
    ld = ":".join(_cmeel_lib_dirs() + [ros_lib, os.environ.get("LD_LIBRARY_PATH", "")])
    set_ld = SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=ld)
    set_plugin = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=":".join([ros_lib, os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")]))
    # So the GUI (gui:=true) can resolve the Go2 visual meshes: the URDF references
    # them as package://go2_description/dae/*, which sdformat turns into
    # model://go2_description/dae/*. Vendored under kontrolem_description/meshes/, that
    # subtree resolves when meshes/ is on the resource path. Harmless headless (no render).
    meshes = os.path.join(desc, "meshes")
    set_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=":".join([meshes, os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")]))

    gazebo_headless = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration("gui")),
        cmd=["ign", "gazebo", "-s", "-r", "-v", "3", world_path], output="screen")
    gazebo_gui = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("gui")),
        cmd=["ign", "gazebo", "-r", "-v", "3", world_path], output="screen")

    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-world", "go2", "-file", gz_urdf_path,
                   "-name", "go2", "-z", base_height],
        output="screen")

    # M8 bridges (gz -> ROS). /imu/data feeds the BaseEstimatorController; the Go2's
    # IMU sensor (imu link) publishes on the gz topic imu/data. /base_truth_odom is the
    # OdometryPublisher ground truth used only to validate the estimate (never in the
    # control loop). Both are one-way gz->ROS ([ = read-from-gz).
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=[
            "/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/base_truth_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ])

    # RSP publishes the GZ URDF so gz_ros2_control discovers the hardware + joints.
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[{"robot_description": gz_urdf}])

    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    wbc = Node(package="controller_manager", executable="spawner",
               arguments=["kontrolem_controller", "-c", "/controller_manager"])
    # M8 estimator: runs alongside the WBC, publishing /base_odom. It shares the joint +
    # contact STATE interfaces (read-only, so multiple controllers may claim them). In
    # base_source:=estimate it becomes the WBC's base source; in ecm it just publishes
    # for validation. Auto-enabled whenever base_source is estimate.
    want_est = LaunchConfiguration("estimator").perform(context)
    run_est = (want_est == "true") or (base_source == "estimate")
    on_exit = [jsb, wbc]
    if run_est:
        on_exit.append(Node(package="controller_manager", executable="spawner",
                            arguments=["base_estimator", "-c", "/controller_manager"]))
    # Activate the WBC as fast as possible (in parallel with jsb): the Go2 sags under
    # gravity during the spawn->activation gap, so every saved second means less sag.
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=on_exit))

    return [set_ld, set_plugin, set_resource,
            gazebo_headless, gazebo_gui, rsp, spawn, bridge, after_spawn]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("estimator", default_value="true"),
        DeclareLaunchArgument("base_source", default_value="ecm"),
        OpaqueFunction(function=launch_setup),
    ])
