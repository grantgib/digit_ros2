import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Publisher
class DigitCmdVelPublisher(Node):
    def __init__(self,node_name,topic_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Twist, topic_name, 10)

# Main
def main(args=None):
    # Create Subscriber Nodes
    rclpy.init(args=args)
    node_name = 'pub_cmd_vel'
    topic_name = 'autonomous_cmd_vel'
    node_cmd_vel = DigitCmdVelPublisher(node_name, topic_name)

    x_vel = 0.0
    y_vel = 0.0
    yaw_vel = 0.2
    while rclpy.ok():
        msg_cmd_vel = Twist()
        yaw_vel = yaw_vel * -1.0

        msg_cmd_vel.linear.x = x_vel
        msg_cmd_vel.linear.y = y_vel
        msg_cmd_vel.angular.z = yaw_vel
        node_cmd_vel.publisher_.publish(msg_cmd_vel)
        node_cmd_vel.get_logger().info('\n\
            --> Sending Cmd Vel\n\
                yaw_vel = %f, x_vel = %f, y_vel = %f\n' % (yaw_vel,x_vel,y_vel))
        time.sleep(5)

if __name__ == '__main__':
    main()
