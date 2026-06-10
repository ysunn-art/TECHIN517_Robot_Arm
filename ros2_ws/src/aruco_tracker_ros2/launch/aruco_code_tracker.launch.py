import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_tracker_ros2')
    default_config = os.path.join(pkg_share, 'config', 'aruco_code_tracker.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to parameter YAML file',
    )

    marker_frame_id_arg = DeclareLaunchArgument(
        'marker_frame_id',
        default_value='aruco_code',
        description='TF frame name (or prefix) for the tracked marker(s)',
    )

    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='-1',
        description='ArUco marker ID to track (-1 = track all)',
    )

    node = Node(
        package='aruco_tracker_ros2',
        executable='aruco_code_tracker',
        name='aruco_code_tracker',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'marker_frame_id': LaunchConfiguration('marker_frame_id'),
                'marker_id': LaunchConfiguration('marker_id'),
            },
        ],
    )

    return LaunchDescription([
        config_arg,
        marker_frame_id_arg,
        marker_id_arg,
        node,
    ])
