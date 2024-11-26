from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

def generate_launch_description():

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='582',
        description='Marker ID. '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.05',
        description='Marker size in m. '
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Frame in which the marker pose will be refered. '
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame', default_value='camera_frame',
        description='Frame in which the marker pose will be refered. '
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement. ',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
    )

    camera_config_file = PathJoinSubstitution(
        [FindPackageShare("ball_detection"),"config","zed2_params.yaml"]
    )

    aruco_single_params = {
        'image_is_rectified': False,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'marker_frame': LaunchConfiguration('marker_frame'),
        'camera_frame': LaunchConfiguration('camera_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
    }

    args = []
    args.append(marker_id_arg)
    args.append(marker_size_arg)
    args.append(marker_frame_arg)
    args.append(camera_frame_arg)
    args.append(reference_frame)
    args.append(corner_refinement_arg)

    return LaunchDescription(args + [
        Node(
            package='usb_cam',
            namespace='camera',
            executable='usb_cam_node_exe',
            parameters=[camera_config_file],
            name='cam',
            output='screen',
            remappings=[('/camera/image_raw','/out/image_raw'),
                        ('/camera/camera_info','/out/camera_info')]
        ),
        # Node(
        #     package='image_proc',
        #     executable='crop_decimate_node',
        #     parameters=[{'width': 1920}]
        # ),
        Node(
            package='aruco_ros',
            executable='single',
            parameters=[aruco_single_params],
            remappings=[('/image','/out/image_raw'),
                        ('/camera_info','/out/camera_info')]
        )
    ])
