from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='digit_json',
            executable='cmd_vel_subscriber',
            output='screen',
            arguments=['real']
        ),
    ])