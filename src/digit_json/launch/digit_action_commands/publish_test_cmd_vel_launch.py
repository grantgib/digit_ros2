from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='digit_json',
            executable='test_pub_cmd_vel',
            output='screen',
        ),
    ])