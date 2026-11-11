"""whill_odometry を単体で起動する（パラメータはコードの既定値。車椅子 red の motor_speed_ts を購読する）.

  ros2 launch whill_odometry odometry.launch.py
  ros2 launch whill_odometry odometry.launch.py input_topic_name:=/Drp5_whill_blue/whill_node/motor_speed_ts

自己位置推定と一緒に起動するときは localization_bringup（env:=<色> で param/<色>/whill_odometry.yaml を読む）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic_name', default_value='/Drp5_whill_red/whill_node/motor_speed_ts',
            description='motor_speed_ts of the wheelchair (Drp5_whill_<color>)'),
        Node(
            package='whill_odometry',
            executable='whill_odometry_node',
            name='whill_odometry',
            parameters=[{'input_topic_name': LaunchConfiguration('input_topic_name')}],
            output='screen',
        ),
    ])
