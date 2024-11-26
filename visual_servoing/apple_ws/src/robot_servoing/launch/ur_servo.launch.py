import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="0.0.0.0",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("ur_robot_driver"), "/launch/ur_control.launch.py"]),
        launch_arguments={
            "ur_type": "ur5",
            "launch_rviz": "false",
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "use_mock_hardware": use_fake_hardware,
            "fake_sensor_commands": use_fake_hardware,
            "activate_joint_controller": "false",
        }.items()
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("ur_moveit_config"), "/launch/ur_moveit.launch.py"]),
        launch_arguments={
            "ur_type": "ur5",
            "launch_rviz": "true",
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware
        }.items(),
    )

    return LaunchDescription(declared_arguments + 
        [
            ur_control_launch,
            ur_moveit_launch
        ]
    )