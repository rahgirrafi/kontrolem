# Shared launch body for the Kontrol'Em sim demos. Both cart_pole.launch.py and
# arm2.launch.py are thin wrappers over build_sim_launch() — they differ only in
# which URDF + controller config they load.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _cmeel_lib_dirs():
    """Locate the cmeel Pinocchio prefix's shared-library dirs, if present.

    The core links Pinocchio from the pip 'cmeel' wheel (see DEVELOPMENT.md D1);
    those libs (Pinocchio + its bundled Boost) are not on the default loader
    path, so ros2_control_node fails to dlopen our plugins without them. We
    prepend them here so `ros2 launch` works without a manual LD_LIBRARY_PATH
    export. If cmeel is absent (e.g. apt Pinocchio), this is a harmless no-op.
    """
    try:
        import importlib.util

        spec = importlib.util.find_spec("cmeel")
        if not spec or not spec.origin:
            return []
        prefix = os.path.normpath(
            os.path.join(os.path.dirname(spec.origin), os.pardir, "cmeel.prefix"))
        return [d for d in (os.path.join(prefix, "lib"), os.path.join(prefix, "lib64"))
                if os.path.isdir(d)]
    except Exception:
        return []


def build_sim_launch(urdf_file, controllers_yaml, controller_name="kontrolem_controller"):
    """Compose the standard sim launch: robot_state_publisher + ros2_control_node
    (with the URDF for the resource manager AND the controller), joint state
    broadcaster, then the control law once the broadcaster is up.

    urdf_file / controllers_yaml are basenames inside kontrolem_description/urdf
    and kontrolem_bringup/config respectively.
    """
    description_pkg = get_package_share_directory("kontrolem_description")
    bringup_pkg = get_package_share_directory("kontrolem_bringup")

    urdf_path = os.path.join(description_pkg, "urdf", urdf_file)
    with open(urdf_path, "r") as f:
        robot_description_xml = f.read()
    robot_description = {"robot_description": robot_description_xml}
    controllers_path = os.path.join(bringup_pkg, "config", controllers_yaml)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            controllers_path,
            robot_description,  # resource manager (sim hardware) reads this
            {controller_name + ".robot_description": robot_description_xml},  # our controller
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    control_law_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager", "/controller_manager"],
    )

    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[control_law_spawner],
        )
    )

    ld = ":".join(_cmeel_lib_dirs() + [os.environ.get("LD_LIBRARY_PATH", "")])
    set_ld_path = SetEnvironmentVariable(name="LD_LIBRARY_PATH", value=ld)

    return LaunchDescription([
        set_ld_path,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_controller,
    ])
