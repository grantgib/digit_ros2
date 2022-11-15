import sys
import asyncio
import agility
import agility.messages as msgs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

# Odometry Publisher
class BaseOdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_base_pub_node')
        self.publisher_ = self.create_publisher(Odometry, 'odometry/filtered', 10)

# TF Base Publisher
class BaseTransformPublisher(Node):
    def __init__(self):
        super().__init__('tf_base_pub_node')
        self.br = TransformBroadcaster(self)

# TF Base Footprint Publisher
class BaseFootprintTransformPublisher(Node):
    def __init__(self):
        super().__init__('tf_base_footprint_pub_node')
        self.br = TransformBroadcaster(self)

# Main Loop
async def main_async(ip_address, publish_period, pub_odom, pub_tf_base, pub_tf_base_footprint):
    async with agility.JsonApi(address=ip_address) as api:

        # Get responses for a group of simultaneous queries
        queries = [
            "get-timestamp",
            ["get-object",{"object": {"special-frame": "world"}}],
            ["get-object",{"object": {"robot-frame": "base"}}],
            ["get-object-kinematics",
                {
                "object": {"robot-frame": "base"},
                 "relative-to": {"special-frame": "world"},
                 "representation": "qxyz"
                }]
        ]
        
        query_period = publish_period # seconds
        async with api.periodic_query(queries, query_period) as q:
            async for results in q:

                # Parse query results
                base_pose = results[3].transform.qxyz
                base_quat = base_pose[0:4]
                base_pos = base_pose[4:7]
                base_vel = results[3].velocity.rpyxyz
                base_angvel = base_vel[0:3]
                base_linvel = base_vel[3:6]
                # print(
                #     f"run-time: {results[0].run_time}, "
                #     f"unix-time: {results[0].unix_time}, \n \n"
                #     f"name: {results[1].attributes.name}, "
                #     f"id: {results[1].id}, \n"
                #     f"name: {results[2].attributes.name}, "
                #     f"id: {results[2].id}, \n \n"
                #     # f"pose (qxyz): {base_pose}, \n"
                #     f"quat: {base_quat}, \n"
                #     f"pos: {base_pos}, \n"
                #     # f"velocity: {base_vel}, \n"
                #     f"linvel: {base_linvel}, \n"
                #     f"angvel: {base_angvel}, \n \n"
                # )

                # Base Transform
                msg_tf_base = TransformStamped()
                msg_tf_base.header.stamp = pub_tf_base.get_clock().now().to_msg()
                msg_tf_base.header.frame_id = 'odom'

                msg_tf_base.child_frame_id = 'base_link'

                msg_tf_base.transform.translation.x = base_pos[0]
                msg_tf_base.transform.translation.y = base_pos[1]
                msg_tf_base.transform.translation.z = base_pos[2]

                msg_tf_base.transform.rotation.w = base_quat[0]
                msg_tf_base.transform.rotation.x = base_quat[1]
                msg_tf_base.transform.rotation.y = base_quat[2]
                msg_tf_base.transform.rotation.z = base_quat[3]

                pub_tf_base.br.sendTransform(msg_tf_base)

                # Base w/ Footprint Transform
                msg_tf_base_footprint = msg_tf_base

                msg_tf_base_footprint.header.stamp = pub_tf_base_footprint.get_clock().now().to_msg()
                msg_tf_base_footprint.header.frame_id = 'base_link'

                msg_tf_base_footprint.child_frame_id = 'base_footprint'

                msg_tf_base_footprint.transform.translation.z = 0.0

                pub_tf_base_footprint.br.sendTransform(msg_tf_base_footprint)

                # Odometry
                OdomMsg = Odometry()
                OdomMsg.header.stamp = pub_odom.get_clock().now().to_msg()
                OdomMsg.header.frame_id = 'odom'

                OdomMsg.child_frame_id = 'base'

                OdomMsg.pose.pose.position.x = base_pos[0]
                OdomMsg.pose.pose.position.y = base_pos[1]
                OdomMsg.pose.pose.position.z = base_pos[2]

                OdomMsg.pose.pose.orientation.w = base_quat[0]
                OdomMsg.pose.pose.orientation.x = base_quat[1]
                OdomMsg.pose.pose.orientation.y = base_quat[2]
                OdomMsg.pose.pose.orientation.z = base_quat[3]
                
                OdomMsg.twist.twist.linear.x = base_linvel[0]
                OdomMsg.twist.twist.linear.y = base_linvel[1]
                OdomMsg.twist.twist.linear.z = base_linvel[2]

                OdomMsg.twist.twist.angular.x = base_angvel[0]
                OdomMsg.twist.twist.angular.y = base_angvel[1]
                OdomMsg.twist.twist.angular.z = base_angvel[2]
                
                pub_odom.publisher_.publish(OdomMsg)

def main(args =None):
    # Parse Arguments
    ip_address = sys.argv[1]
    publish_rate = float(sys.argv[2])

    # ROS Node initializations
    rclpy.init(args=args)
    pub_odom = BaseOdometryPublisher()
    pub_tf_base = BaseTransformPublisher()
    pub_tf_base_footprint = BaseFootprintTransformPublisher()

    # Start Publish Loop
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main_async(ip_address, publish_rate, pub_odom, pub_tf_base, pub_tf_base_footprint))

# Entry
if __name__ == "__main__":
    main()

    
