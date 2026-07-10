# M6.3 base-state round-trip against Gazebo. Base pose/twist comes from Gazebo's
# ground-truth odometry-publisher (NOT our custom sim, and NOT gz_ros2_control):
# Gazebo -> ign /base_odom -> ros_gz_bridge -> ROS /base_odom -> OdometryBaseBridge
# (a ros2_control sensor) -> the 13 floating_base state interfaces -> FloatingStateProbe
# reassembles a manifold State and checks it is SE(3)-consistent with the producer.
# There is no IgnitionSystem here: the controller_manager runs standalone and its
# only hardware is the topic bridge.
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from _common import _cmeel_lib_dirs  # noqa: E402


def generate_launch_description():
    desc = get_package_share_directory("kontrolem_description")
    bringup = get_package_share_directory("kontrolem_bringup")

    world = os.path.join(desc, "worlds", "floating_box.world.sdf")
    urdf_path = os.path.join(desc, "urdf", "floating_box_gz.urdf")
    with open(urdf_path) as f:
        urdf = f.read()
    rd = {"robot_description": urdf}
    controllers = os.path.join(bringup, "config", "floating_box_gz_controllers.yaml")

    # Gazebo headless + running (-s server-only, -r run on start).
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", "-v", "2", world], output="screen")

    # ign Odometry -> ROS nav_msgs/Odometry on /base_odom.
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/base_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry"])

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[rd])

    cm = Node(package="controller_manager", executable="ros2_control_node", output="screen",
              parameters=[controllers, rd,
                          {"floating_state_probe.robot_description": urdf}])

    probe = Node(package="controller_manager", executable="spawner",
                 arguments=["floating_state_probe", "-c", "/controller_manager"])
    # Give Gazebo + the bridge + odometry time to come up before activating the probe.
    delay_probe = TimerAction(period=6.0, actions=[probe])

    ld = ":".join(_cmeel_lib_dirs() + ["/opt/ros/humble/lib", os.environ.get("LD_LIBRARY_PATH", "")])
    return LaunchDescription([
        SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=ld),
        SetEnvironmentVariable(name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH", value="/opt/ros/humble/lib"),
        gazebo, bridge, rsp, cm, delay_probe])
