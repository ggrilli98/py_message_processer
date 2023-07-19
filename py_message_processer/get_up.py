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


class get_up(Node):
    def __init__(self):
        super().__init__('get_up')



        self.timer_send = self.create_timer(3, self.send_pick_up_movement)
        self.get_up_publisher = self.create_publisher(Float32MultiArray, 'reachstacker_geom_values', 10) 
        self.publisher_spreader = self.create_publisher(ECSpreaderControl, '/reachstacker/spreader_control', 10)


    def send_pick_up_movement(self):
        geom_values = Float32MultiArray()
        geom_values.data = [35.0, 80.0, 0.0, 0.0, 0.0]
        self.publisher_geometrical.publish(geom_values)
        print("mandato valori", geom_values.data)

    

def main(args=None):
    rclpy.init(args=args)
    spreader_msg_publisher = get_up()
    rclpy.spin(spreader_msg_publisher)
    spreader_msg_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()