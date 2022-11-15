import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Publisher
class DigitCmdVelPublisher(Node):
    def __init__(self,node_name,topic_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(PoseStamped, topic_name, 10)

# Main
def main(args=None):
    # Create Subscriber Nodes
    rclpy.init(args=args)
    node_name = 'pub_cmd_goto'
    topic_name = 'autonomous_cmd_goto'
    node_cmd_vel = DigitCmdVelPublisher(node_name, topic_name)

    qw = 1.0
    qx = 0.0
    qy = 0.0
    qz = 0.0

    # qw = 0.9980784190206976
    # qx = -0.0015440523568275631
    # qy = 0.0007963646651961521
    # qz = -0.06193909258928553

    x_pose = 1.0
    y_pose = 0.2
    while rclpy.ok():
        msg_goto = PoseStamped()
        y_pose *= -1.0
        msg_goto.pose.orientation.w = qw
        msg_goto.pose.orientation.x = qx
        msg_goto.pose.orientation.y = qy
        msg_goto.pose.orientation.z = qz
        msg_goto.pose.position.x = x_pose
        msg_goto.pose.position.y = y_pose
        node_cmd_vel.publisher_.publish(msg_goto)
        node_cmd_vel.get_logger().info('\n\
            --> Sending ActionGoTo\n\
                qw = %f, qx = %f, qy = %f, qz = %f\n\
                x = %f, y = %f' % (qw,qx,qy,qz,x_pose,y_pose))
        time.sleep(10)

if __name__ == '__main__':
    main()