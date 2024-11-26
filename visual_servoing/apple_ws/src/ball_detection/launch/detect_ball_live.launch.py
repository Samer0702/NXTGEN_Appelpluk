from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    camera_config_file = PathJoinSubstitution(
        [FindPackageShare("ball_detection"),"config","cam_params.yaml"]
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            namespace='camera',
            executable='usb_cam_node_exe',
            parameters=[camera_config_file],
            name='cam',
            output='screen'
        ),
        Node(
            package='ball_detection',
            namespace='detection',
            executable='ball_detector',
            name='ball_detector',
            output='screen'
        )
    ])