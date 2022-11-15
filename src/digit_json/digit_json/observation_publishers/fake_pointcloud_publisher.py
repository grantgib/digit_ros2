# from enum import Enum
# from time import perf_counter

# import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

# Point Cloud Publisher
class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('PointCloud2_pub_node')
        self.publisher_ = self.create_publisher(PointCloud2, 'upper_velodyne_vlp16/depth/points', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create and Update PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'base_link'
        cloud_msg.height = 1
        cloud_msg.width = 1  # usually a large value
        cloud_msg.is_bigendian = False  
        cloud_msg.point_step = 16
        cloud_msg.row_step = 1  # usually a large value
        cloud_msg.is_dense = True

        # Update Fields
        fields = []
        pf_x = PointField()
        pf_x.name = "x"
        pf_x.datatype = 7
        pf_x.offset = 0
        pf_x.count = 1  
        
        pf_y = PointField()
        pf_y.name = "y"
        pf_y.datatype = 7
        pf_y.offset = 4
        pf_y.count = 1 

        pf_z = PointField()
        pf_z.name = "z"
        pf_z.datatype = 7
        pf_z.offset = 8
        pf_z.count = 1

        pf_intensity = PointField()
        pf_intensity.name = "intensity"
        pf_intensity.datatype = 7
        pf_intensity.offset = 12
        pf_intensity.count = 1

        fields.append(pf_x)
        fields.append(pf_y)
        fields.append(pf_z)
        fields.append(pf_intensity)
        cloud_msg.fields = fields

        # No data to publish
        # cloud_msg.data = depth_points.tobytes()  # why in bytes

        # Publish message
        self.publisher_.publish(cloud_msg)
           

# Main
def main(args=None):
    rclpy.init(args=args)
    pointcloud_publisher = PointCloudPublisher()
    pointcloud_publisher.get_logger().info("Publisher Fake PointCloud2 Message")
    rclpy.spin(pointcloud_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pointcloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
