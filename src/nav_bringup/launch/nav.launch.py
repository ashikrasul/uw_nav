"""The whole nav stack. To add a node: append one row to NODES and put its YAML in config/."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

SHARE = get_package_share_directory('nav_bringup')

# (package, executable, node name, YAML files in config/, topic remappings)
NODES = [
    # drivers
    ('nav_drivers', 'gps_fix', 'gps_publisher', ['gps_fix.yaml'], []),
    ('nav_drivers', 'imu_node', 'imu_publisher', ['imu_config.yaml'], []),
    ('nav_drivers', 'acoustic_node', 'acoustic_ranger',
     ['beacons.yaml', 'acoustic_config.yaml'], []),
    # state estimation (robot_localization)
    ('robot_localization', 'ekf_node', 'ekf_filter_node', ['ekf.yaml'], []),
    ('robot_localization', 'navsat_transform_node', 'navsat_transform', ['navsat.yaml'],
     [('imu', 'imu/data')]),
    # beacon positions
    ('nav_localization', 'beacon_locator', 'beacon_locator',
     ['beacons.yaml', 'beacon_locator.yaml'], []),
    # live view: Foxglove connects to ws://<agent-ip>:8765
    ('foxglove_bridge', 'foxglove_bridge', 'foxglove_bridge', ['foxglove_bridge.yaml'], []),
]


def generate_launch_description():
    with open(os.path.join(SHARE, 'urdf', 'agent.urdf')) as f:
        robot_description = f.read()

    nodes = [
        # static TFs base_link -> sensor frames
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
    ]
    for package, executable, name, configs, remappings in NODES:
        nodes.append(Node(
            package=package,
            executable=executable,
            name=name,
            parameters=[os.path.join(SHARE, 'config', c) for c in configs],
            remappings=remappings,
            output='screen',
            respawn=True,          # last resort only: nodes handle hardware errors themselves
            respawn_delay=2.0,
        ))
    return LaunchDescription(nodes)
