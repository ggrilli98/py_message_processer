import os
import math
import time
from typing import List

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from ec_msgs.msg import ECSpreaderControl

# from ackermann_msgs.msg import AckermannDriveStamped
# from ament_index_python import get_resource
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
# from rclpy.qos import QoSProfile
# from apriltag_ros.msg import AprilTagDetectionArray


class msg_processer_node(Node):
    def __init__(self):
        super().__init__('processer_node')

        self.spreader_subscription = self.create_subscription(
            ECSpreaderControl,
            '/reachstacker/spreader_control',
            self.spreader_control_callback,
            10
        )

    #GETS THE MESSAGE SENT FROM THE SPREADER CONTROL
    def spreader_control_callback(self, msg):                   
            for i, name in enumerate(msg.forward):


                time.sleep(5)
            

def main(args=None):
    rclpy.init(args=args)
    processer_node = msg_processer_node()
    rclpy.spin(processer_node)
    processer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

