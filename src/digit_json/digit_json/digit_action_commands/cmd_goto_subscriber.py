from ws4py.client.threadedclient import WebSocketClient
import json
import time
import sys 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# Websocket Class
class BasicClient(WebSocketClient):
    def opened(self):
        self.operation_mode = None
        self.responded = True

        privilege_request = ['request-privilege', 
                                {'privilege' : 'change-action-command',
                                 'priority' : 0}]
        self.send(json.dumps(privilege_request))

    def closed(self, code, reason):
        print(("Closed", code, reason))

    def received_message(self, m):
        dataloaded = json.loads(m.data)
        message_type = str(dataloaded[0])
        message_dict = dataloaded[1]

        if message_type == 'privileges':
            self.done = message_dict['privileges'][0]['has']
            if self.done:
                print("Privilege request executed successfully.")
        elif message_type == 'robot-status':
            self.responded = True
            self.operation_mode = str(message_dict['operation-mode'])
        elif message_type == 'error':
            self.error_info = str(message_dict['info'])
            print('Error: ', self.error_info)
        elif message_type == 'action-status-changed':
            if message_dict['status'] == 'running':
                # print('Command received and is running')
                self.completed = False
            elif message_dict['status'] == 'success':
                self.completed = True
        elif message_type == 'object-kinematics':  
            self.arm_pose = message_dict['transform']['rpyxyz'] 

# Commmand GoTo Subscriber
class DigitCmdGotoSubscriber(Node):
    def __init__(self,node_name,topic_name):
        # Initialize Node
        super().__init__(node_name)

        # Secure Websocket Connection
        if sys.argv[1] == 'sim':
            self.ws = BasicClient('ws://127.0.0.1:8080', protocols=['json-v1-agility'])
            self.ws.daemon = False
        elif sys.argv[1] == 'real':
            self.ws = BasicClient('ws://10.10.1.1:8080', protocols=['json-v1-agility'])
            self.ws.daemon = False
        else:
            sys.exit("Need to provide argument 'sim' or 'real'")
        
        while True:
            try:
                self.ws.connect()
                print("WS connection established")
                time.sleep(1)
                break
            except:
                print('WS connection NOT established')
                time.sleep(1)

        # Create Subscription
        self.subscription = self.create_subscription(PoseStamped, topic_name, self.cmd_goto_callback,10)
    
    def cmd_goto_callback(self,msg):
        # Parse message
        qw = msg.pose.orientation.w # world frame
        qx = msg.pose.orientation.x 
        qy = msg.pose.orientation.y 
        qz = msg.pose.orientation.z 
        x_pos = msg.pose.position.x # world
        y_pos = msg.pose.position.y # world
        self.get_logger().info('\n\
            --> ActionGoto Command Received\n\
                qw = %f, qx = %f, qy = %f, qz = %f,\n\
                x = %f, y = %f\n' % (qw,qx,qy,qz,x_pos,y_pos))
        # Create and Send json command
        json_msg = [
            "action-goto",
            {
                "target": {"qxy":[qw,qx,qy,qz,x_pos,y_pos]},
                "heading-constraint": None,
                "reference-frame": {"special-frame": "world"},
                "position-tolerance": 0.1,
                "orientation-tolerance": 0.1,
                "mobility-parameters": {"avoid-obstacles": False}
            }
        ]
        self.ws.send(json.dumps(json_msg))
        self.get_logger().info('Sent action-goto command')

# Main
def main(args=None):
    # Create Subscriber Nodes
    rclpy.init(args=args)
    node_name = 'sub_cmd_goto'
    topic_name = 'autonomous_cmd_goto'
    node_cmd_vel = DigitCmdGotoSubscriber(node_name, topic_name)

    # Spin node
    rclpy.spin(node_cmd_vel)

    # (Optional) destroy node
    # node_cmd_vel.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
