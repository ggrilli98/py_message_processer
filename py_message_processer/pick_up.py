import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from random import uniform
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_msgs.msg import String 
from std_msgs.msg import Float32MultiArray
from ec_msgs.msg import ECSpreaderControl
from ec_msgs.msg import ECSpreaderStatus
import numpy as np
import time

# from ackermann_msgs.msg import AckermannDriveStamped
# from ament_index_python import get_resource
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
# from rclpy.qos import QoSProfile
# from apriltag_ros.msg import AprilTagDetectionArray



class pick_upper(Node):
    def __init__(self):
        super().__init__('pick_up_node')



        self.timer_send = self.create_timer(3, self.send_pick_up_movement)
        self.pick_up_publisher = self.create_publisher(Float32MultiArray, 'reachstacker_geom_goals', 10) 
        self.publisher_spreader = self.create_publisher(ECSpreaderControl, '/reachstacker/spreader_control', 10)


    def send_pick_up_movement(self):
        geom_values = Float32MultiArray()
        geom_values.data = [21.44, 251.209, 16.0, 0.0, 0.0]
        self.pick_up_publisher.publish(geom_values)
        print("mandato valori", geom_values.data)
        spreader_command = ECSpreaderControl()
        spreader_command.header.stamp = self.get_clock().now().to_msg()
        spreader_command.enable_brake = [True, True, True, True, True, True, True, True]
        spreader_command.forward = [False, False, False, False, False, False, False, True]
        spreader_command.backward = [True, True, True, True, True, True, False, False]
        spreader_command.speed = [255, 255, 255, 255, 255, 255, 0, 255]
        spreader_command.magnet = False
        self.publisher_spreader.publish(spreader_command)


    

def main(args=None):
    rclpy.init(args=args)
    pick_up_pub = pick_upper()
    rclpy.spin(pick_up_pub)
    pick_up_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()