import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('bno085_driver'), 'config', 'bno085.yaml')
    return LaunchDescription([
        # base_link -> imu_link: x=1.0, y=0, z=1.1 m, no rotation
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_link_to_imu_link',
             arguments=['--x', '1.0', '--y', '0', '--z', '1.1',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'imu_link']),
        Node(package='bno085_driver', executable='bno085_node',
             name='bno085_node', output='screen', parameters=[cfg]),
    ])
