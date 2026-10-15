import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('whill_odometry'), 'param', 'odometry.yaml')

    return LaunchDescription([
        Node(
            package='whill_odometry',
            executable='whill_odometry_node',
            name='whill_odometry',
            parameters=[param],
            output='screen',
        ),
    ])
