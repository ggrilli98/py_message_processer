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



class Spreadercontroller(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')


        # PUBLISHERS
        self.publisher_spreader = self.create_publisher(ECSpreaderControl, '/reachstacker/spreader_control', 10)
        self.publisher_geometrical = self.create_publisher(Float32MultiArray, 'reachstacker_geom_values', 10)

        #SUBSCRIBERS
        self.spreader_stops_sub = self.create_subscription(ECSpreaderStatus, 'reachstacker/spreader_status', self.spreader_limitswitches_callback, 10)

        # VARIABLES for the spreader
        self.timer_period_spreader = 0.1  # Publish rate of 10 Hz
        self.timer_spreader_overtime_error = 2 # some spreaders are faulty and do no support having a velocity message published for more than x amount of seconds, x still to be established
        self.spreader_switch = False
        self.minimum_limitswitches = [bool]*8
        self.maximum_limitswitches = [bool]*8

        # VARIABLES for the geometrical transformer 
        # 100 point interpolation 
        self.boom_tilt_deg = np.linspace(0.0, 50.0, 100)
        self.boom_extension_mm = np.linspace(0.0, 500.0, 100)
        self.spreader_pitch_deg = np.linspace(0.0, -50.0, 100)
        self.spreader_yaw_deg = np.linspace(-90.0, 1170.0, 100)
        self.spreader_translation_mm = np.linspace(-5.0, 5.0, 100)
        self.killer_geometrical_values = False

        # VARIABLES for the velocity transformer
        self.w0_0_deg = 6.667
        self.w3_3_deg = 10.0
        self.w5_5_deg = 10.0

        # TIMERS FOR PUBLISHERS AND CONTROL 
        self.timer_spreader = self.create_timer(self.timer_period_spreader, self.publish_spreader_command)
        self.timer_overtime_spreader = self.create_timer(self.timer_spreader_overtime_error, self.spreader_stopper_callback)
        self.timer_geometrical = self.create_timer(1, self.publish_geom)        

        
    # SCRIPT TO CHECK THE VALUES OF THE E-STOPS
    def spreader_limitswitches_callback(self, msg):
        for i in enumerate(msg.min_stop):
            self.minimum_limitswitches[i] = msg.min_stop[i]
            self.maximum_limitswitches[i] = msg.max_stop[i]
            if self.minimum_limitswitches[7] == True or self.maximum_limiswitches[7] == True:
                print('limitswitches triggered')


    # TRIGGER FOR THE OVERTIME MESSAGE
    def spreader_stopper_callback(self):
        if self.spreader_switch == False:
             self.spreader_switch = True    
        else:
             self.spreader_switch = False


    #ARRAY OF 8, FIRST 6 ARE FOR FLIPPERS, 7 MAGNET, 8 EXPAND/COLLAPSE THE SPREADER
    def publish_spreader_command(self):
        if self.spreader_switch == False:  # We are oversending the message
                
            #MESSAGE TO SOP THE SPREADER IN PLACE
            spreader_command = ECSpreaderControl()
            spreader_command.header.stamp = self.get_clock().now().to_msg()
            #                          [flipper1, flipper2, flipper3, flipper4, flipper5, flipper6, magnet, extension]
            spreader_command.forward = [False, False, False, False, False, False, False, False]
            spreader_command.backward = [False, False, False, False, False, False, False, False]
            spreader_command.speed = [0, 0, 0, 0, 0, 0, 0, 0]
            spreader_command.magnet = False
            spreader_command.enable_brake = [True, True, True, True, True, True, True, True]

        if self.spreader_switch == True: #we are not oversending the message
                
                # EXAMPLE MESSAGES

            spreader_command = ECSpreaderControl()
            spreader_command.header.stamp = self.get_clock().now().to_msg()
            spreader_command.enable_brake = [True, True, True, True, True, True, True, True]

            #MESSAGE TO OPEN THE SPREADER
            #                          [flipper1, flipper2, flipper3, flipper4, flipper5, flipper6, magnet, extension]
            spreader_command.forward = [False, False, False, False, False, False, False, True]
            spreader_command.backward = [False, False, False, False, False, False, False, False]
            spreader_command.speed = [0, 0, 0, 0, 0, 0, 0, 255]
            spreader_command.magnet = False

            # #MESSAGE TO CLOSE THE SPREADER
            # #                          [flipper1, flipper2, flipper3, flipper4, flipper5, flipper6, magnet, extension]
            # spreader_command.forward = [False, False, False, False, False, False, False, False]
            # spreader_command.backward = [False, False, False, False, False, False, False, True]
            # spreader_command.speed = [0, 0, 0, 0, 0, 0, 0, 255]
            # spreader_command.magnet = False

            # #MESSAGE TO DISENGAGE THE MAGNET  AAAAAAAAAAAAAAAAAAAAAAAAAAAA  DON'T KNOW FOR CERTAIN, NEEDS TO BE TESTED
            # #                          [flipper1, flipper2, flipper3, flipper4, flipper5, flipper6, magnet, extension]
            # spreader_command.forward = [False, False, False, False, False, False, False, False]
            # spreader_command.backward = [False, False, False, False, False, False, False, False]
            # spreader_command.speed = [0, 0, 0, 0, 0, 0, 0, 0]
            # spreader_command.magnet = True

            # #MESSAGE TO ENGAGE THE MAGNET  AAAAAAAAAAAAAAAAAAAAAAAAAAAA  DON'T KNOW FOR CERTAIN, NEEDS TO BE TESTED
            # #                          [flipper1, flipper2, flipper3, flipper4, flipper5, flipper6, magnet, extension]
            # spreader_command.forward = [False, False, False, False, False, False, False, False]
            # spreader_command.backward = [False, False, False, False, False, False, False, False]
            # spreader_command.speed = [0, 0, 0, 0, 0, 0, 0, 0]
            # spreader_command.magnet = False

            # #MESSAGE TO RAISE THE FLIPPERS
            # #                          [flipper1, flipper2, flipper3, flipper4, flipper5, flipper6, magnet, extension]
            # spreader_command.forward = [True, True, True, True, True, True, False, False]
            # spreader_command.backward = [False, False, False, False, False, False, False, False]
            # spreader_command.speed = [255, 255, 255, 255, 255, 255, 0, 0]
            # spreader_command.magnet = False
        
            # #MESSAGE TO LOWER THE FLIPPERS 
            # #                          [flipper1, flipper2, flipper3, flipper4, flipper5, flipper6, magnet, extension]
            # spreader_command.forward = [False, False, False, False, False, False, False, False]
            # spreader_command.backward = [True, True, True, True, True, True, False, False]
            # spreader_command.speed = [255, 255, 255, 255, 255, 255, 0, 0]
            # spreader_command.magnet = False

        self.publisher_spreader.publish(spreader_command)
        # print('forward',spreader_command.forward)
        # print('backward',spreader_command.backward)


    # STUFF FOR THE KINEMATICS AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAS

    # PUBLISHER OF GEOMETRICAL POSITIONS OF THE ROBOT JOINTS FOR THE REFERENCE SYSTEM TRANSFORMATION 
    def publish_geom(self):
        geom_values = Float32MultiArray()
        if self.killer_geometrical_values == False:
            for i in range(100):
                geom_values.data = [self.boom_tilt_deg[i], self.boom_extension_mm[i], self.spreader_pitch_deg[i], self.spreader_yaw_deg[i], self.spreader_translation_mm[i], self.w0_0_deg, self.w3_3_deg, self.w5_5_deg]         
                self.publisher_geometrical.publish(geom_values)
                print("mandato valori", geom_values.data)
                time.sleep(0.05)
            self.killer_geometrical_values = True   

def main(args=None):
    rclpy.init(args=args)
    spreader_msg_publisher = Spreadercontroller()
    rclpy.spin(spreader_msg_publisher)
    spreader_msg_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()