from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='digit_json',
            executable='fake_pointcloud_publisher',
            name='Fake_PointCloud2_pub_node',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','map','pointcloud_frame']
        )
    ])