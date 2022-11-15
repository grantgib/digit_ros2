from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ip_address = '127.0.0.1'
    publish_rate = '0.05' # seconds
    return LaunchDescription([
        Node(
            package='digit_json',
            executable='state_publisher',
            output='screen',
            arguments=[ip_address,publish_rate]
        ),
    ])