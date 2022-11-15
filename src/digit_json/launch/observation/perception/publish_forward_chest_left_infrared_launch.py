from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sensor = 'forward_chest_left_infrared'
    node_name = sensor + '_pub_node'
    topic_name = sensor
    frame_name = sensor + '_frame'
    return LaunchDescription([
        Node(
            package='digit_json',
            executable='image_publisher',
            name=node_name,
            output='screen',
            arguments=[node_name,topic_name,frame_name]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','map', frame_name]
        )
    ])