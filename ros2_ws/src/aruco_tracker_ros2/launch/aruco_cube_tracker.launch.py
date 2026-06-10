import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_tracker_ros2')
    default_config = os.path.join(pkg_share, 'config', 'aruco_cube_tracker.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to parameter YAML file',
    )

    cube_frame_id_arg = DeclareLaunchArgument(
        'cube_frame_id',
        default_value='aruco_cube',
        description='TF frame name for the cube center',
    )

    node = Node(
        package='aruco_tracker_ros2',
        executable='aruco_cube_tracker',
        name='aruco_cube_tracker',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'cube_frame_id': LaunchConfiguration('cube_frame_id')},
        ],
    )

    return LaunchDescription([
        config_arg,
        cube_frame_id_arg,
        node,
    ])
