import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('nav_pkg'), 'config')

    return LaunchDescription([
        Node(
            package='nav_pkg',
            executable='gps_fix',
            name='gps_publisher',
            parameters=[os.path.join(config_dir, 'gps_fix.yaml')],
            output='screen',
            respawn=True,          # restart if the process crashes
            respawn_delay=2.0,
        ),
        Node(
            package='nav_pkg',
            executable='acoustic_node',
            name='acoustic_ranger',
            parameters=[os.path.join(config_dir, 'acoustic_config.yaml')],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package='nav_pkg',
            executable='imu_node',
            name='imu_publisher',
            parameters=[os.path.join(config_dir, 'imu_config.yaml')],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
