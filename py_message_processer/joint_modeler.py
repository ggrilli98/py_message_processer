import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from control_msgs.msg import JointJog
from random import uniform
import time
import numpy as np
from math import cos
from math import sin
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# from ackermann_msgs.msg import AckermannDriveStamped
# from ament_index_python import get_resource
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
# from rclpy.qos import QoSProfile
# from apriltag_ros.msg import AprilTagDetectionArray



    #READ ME PLEASE, THIS IS HOW THE FORWARD KINEMATICS SYSTEM HAS BEEN MODELED:

        #O0 is the origin point of the first spatial system of coordinates, situated at che center of the joint that connects the base of the robot ot the boom, with X_0, Y_0, Z_0 as axis     
        
        # Y_0: THE AXIS PERPENDICULAR TO THE FLANK OF THE ROBOT AND PARALLEL TO THE GROUND
        # X_0: THE AXIS THAT POINTS STRAIGHT OUT FROM THE FRONT OF THE ROBOT, PARALLEL TO THE GOUND                 
        # Z_0: THE AXIS THAT IS PERPENDICULAR FROM THE GROUND, HEADED OUT OF THE GROUND 

        # O1 Is the first robot's coordinate's system origin point, O1 is coincident with O0, axis y_1 runs parallel to the boom of the robot
        #       the rotation between O0 and O1 is, respect to the X_0 axis, and defined by the self.boom_angle_deg variable
        #       the translation between O0 and O1 is none 

        # O2 Is the second robot's coordinate's system origin point, O2 is situated at the beginning of the fixed boom of the robot, axis Y_2 is coincident to Y_1, 
        #       the rotation between O1 and O2 is none
        #       the translation between O1 and O2 is, on the Y_1 axis, the self.distance_tilt_pin_estendor_y variable, adn on the Z_! axis the self.distance_tilt_pin_estendor_z variable

        # O3 Is the third robot's coordinate's system origin point, O3 is situated in the center of the revolute joint that connects the robot's boom and the spreader's pitch motor, axis Y_3 is coincident to Y_2, 
        #       the rotation between O2 and O3 is none
        #       the translation between O2 and O3 is, on the Y_2 axis, the self.boom_extension_mm variable + the self.boom_base_lenght_mm_y variable

        # O4 Is the fourth robot's coordinate's system origin point, O4 is coincident to O3, at the center of the revolute joint
        #       the rotation between O3 and O4 is, respect to the X_3 axis, and defined by the self.spreader_pitch_deg variable
        #       the translation between O3 and O4 is none

        # O5 Is the fifth robot's coordinate's system origin point, O5 is situated at the center of the revolute joint for the robot's spreader yaw motor,
        #       the rotation between O4 and O5 is none
        #       the translation between O4 and O5 is, on the Z_4 axis, and defined by the self.spreader_pitch_to_yaw_distance_mm variable

        # O6 Is the sixth robot's coordinate's system origin point, O6 is coincident with 05
        #       the rotation between O5 and O6 is, respect to the Z_5 axis, the self.spreader_yaw_deg variable
        #       the translation between O5 and O6 is none 

        # O7 Is the seventh robot's coordinate's system origin point, O7 is located in the center of the innest platofrm (for the spreader)
        #       the rotation between O6 and O7 is none
        #       the translation between O6 and O7 is, on the X_6 axis, and defined by the self.spreader_translation_mm variable adn on the Z_6 axis defined by the  self.yaw_joint_plate_distance_z variable

        # O8 Is the eight robot's coordinate's system origin point, O8 is located in the contact point of the magnet to a possible container
        #       the rotation between O7 and O8 is none
        #       the translation between O7 and O8 is, on the X_7 axis,Y_7 axis adn Z_7 axis

class joint_modeler_node_type(Node):
    def __init__(self):
        super().__init__('joints_model')

        # Known workspace variables
        self.pi = math.pi
        self.distance_tilt_pin_estendor_z = 232.86                    # Distance on the Z_1 axis of the boom tilt pin to the extension_joint
        self.distance_tilt_pin_estendor_y = 170.75                    # Distance on the Y_1 axis of the boom tilt pin to the extension_joint
        self.boom_base_lenght_mm_y = 607.89                           # Base boom lenght, to add to the extension, on the Y_2 axis
        self.boom_base_to_pitcher_z = -83.73                          # Distance on the Z_2 axis from the boom extender to the pitcher joint
        self.spreader_pitch_to_yaw_distance_mm_z = -78.0              # Z_4 axis distance between the spreader pitch joint and the spreader yaw joint
        self.yaw_joint_plate_distance_z = -75                         # Distance on the Z_6 axis from the yaw joint to the center of the plate spreader innest
        self.distance_innest_to_magnet_mm_z = -100.0                  # Distance on the Z_7 axis from the the center of the platfrom to the magnet
        self.distance_innest_to_magnet_mm_y = 1.8                     # Distance on the Y_7 axis from the the center of the platfrom to the magnet
        self.distance_innest_to_magnet_mm_x = 4.5                     # Distance on the X_7 axis from the the center of the platfrom to the magnet
        
        self.coordinates_to_transform = [0.0, 0.0, 0.0, 1.0]
        self.i = 0

        #Variables that will be grabbed from the received messages
        self.encoder_values_received = [0.0]*5
        self.boom_angle_deg = 0.0                                   # X axis roation of the boom
        self.boom_extension_mm = 0.0                                # Y axis extension of the boom
        self.spreader_pitch_deg = 0.0                               # X axis rotation of the spreader
        self.spreader_yaw_deg = 0.0                                 # Z axis roation of the spreader
        self.spreader_translation_mm = 0.0                          # Z axis movement of the spreader
        self.w0_0 = 0.0                                             # Angular speed of the boom tilt revolute joint
        self.w3_3 = 0.0                                             # Angular speed of the pitcher revolute joint    
        self.w5_5 = 0.0                                             # Angular speed of the plate yaw revolute joint 

        #SSTORAGE VARIABLES for the final 3dplot
        # O8 position (magnet position)
        self.x_O8 = [0.0]*100
        self.y_O8 = [0.0]*100
        self.z_O8 = [0.0]*100

        # O7 position
        self.x_O7 = [0.0]*100
        self.y_O7 = [0.0]*100
        self.z_O7 = [0.0]*100

        # O6 position (coincident to the O5 position)
        self.x_O5 = [0.0]*100
        self.y_O5 = [0.0]*100
        self.z_O5 = [0.0]*100

        # O4 position (coincident to the O3 position)
        self.x_O3 = [0.0]*100
        self.y_O3 = [0.0]*100
        self.z_O3 = [0.0]*100

        # O2 position 
        self.x_O2 = [0.0]*100
        self.y_O2 = [0.0]*100
        self.z_O2 = [0.0]*100

        # O1 position (coincident to the O0 position)
        # doesn't make sense, it's just static in the orgin, but if it moved could be nice to implemet it too,so why not?
        self.x_O0 = [0.0]*100
        self.y_O0 = [0.0]*100
        self.z_O0 = [0.0]*100

        # VARIABLES for robot-real-data parallel time elaboration
        self.w01_0_real = 0.0
      
        # coefficients for transformation between encoder and real values
        self.axis_coefficients_encoder_to_geometrical = [0.278688524590164, 0.974258133714694, -0.9615, 1.0, 0.1] 
        # intercepts for the linear interpolation
        self.axis_intercepts_encoder_to_geometrical = [ 0.27869, -0.5864, -0.9615, 2.0, 0.0]
        # coefficients for transformation between real values and encoder
        self.axis_coefficients_geometrical_to_encoder = [3.58823529411765, 1.02642201834862, -1.04, 1.0, 10.0]
        # intercepts for the linear interpolation
        self.axis_intercepts_geometrical_to_encoder = [-1.0, +0.6, -1.0, -2.0, 0.0] 

        # Timers
        self.timer_plot = self.create_timer(1, self.plot_3d_data)

        # Subscribers
        self.geom_subsc = self.create_subscription(Float32MultiArray, 'geom_values', self.geom_values_callback, 10)
        self.reach_vel_subsc = self.create_subscription(JointState, '/reachstacker/joint_state', self.real_val_encoder_callback, 10)

        # Publishers for the plotjuggler plot
        self.plotjuggler_O0_publisher = self.create_publisher(Float32MultiArray, 'O0_position', 10)
        self.plotjuggler_O2_publisher = self.create_publisher(Float32MultiArray, 'O2_position', 10)
        self.plotjuggler_O3_publisher = self.create_publisher(Float32MultiArray, 'O3_position', 10)
        self.plotjuggler_O5_publisher = self.create_publisher(Float32MultiArray, 'O5_position', 10)
        self.plotjuggler_O7_publisher = self.create_publisher(Float32MultiArray, 'O7_position', 10)
        self.plotjuggler_O8_publisher = self.create_publisher(Float32MultiArray, 'O8_position', 10)
        self.plotjuggler_vel_publisher = self.create_publisher(Float32MultiArray, 'velocity', 10)

    def encoder_from_geom_array_transformer(self, geom_value_array):
        encoder_value_array = [0.0]*5
        print('diocane')
        for i in range(5):
            encoder_value_array[i] =  geom_value_array[i] * self.axis_coefficients_geometrical_to_encoder[i] + self.axis_intercepts_geometrical_to_encoder[i] 
        return(encoder_value_array)
    
    def geom_from_encoder_array_transformer(self, encoder_value_array):
        print('diomerda')
        geom_value_array = [0.0]*5
        for i in range(5):
            geom_value_array[i] = encoder_value_array[i] * self.axis_coefficients_encoder_to_geometrical[i] + self.axis_intercepts_encoder_to_geometrical[i]
        return(geom_value_array)


    # Homogeneous transformations definitons
    def Homogen_transf(self, R_XYZ, vector):
        Hm_tras = np.block([[R_XYZ, vector],
                             [0.0, 0.0, 0.0, 1.0]])
        return(Hm_tras)        
    
    # Rotational transformation definitons       
    def get_R_x(self, x_angle): 
        R_x =  np.array([[1.0,         0.0,           0.0 ],
                         [0.0, cos(x_angle), -sin(x_angle)],
                         [0.0, sin(x_angle),  cos(x_angle)]])
        return(R_x)
 
    def get_R_y(self, y_angle): 
        R_y =  np.array([[cos(y_angle), 0.0,   sin(y_angle)],
                         [0.0,             1.0        , 0.0],
                         [-sin(y_angle), 0.0,  cos(y_angle)]])
        return(R_y)

    def get_R_z(self, z_angle): 
        R_z =  np.block([[cos(z_angle), -sin(z_angle), 0.0],
                         [sin(z_angle), cos(z_angle),  0.0],
                         [0.0,             0.0,        1.0]])
        return(R_z)
    
    # SKEW DEFINITION
    
    def Skew(self, vector):
        Skew_matrix = np.array([[0.0, -vector[2], vector[1]],
                                [vector[2], 0.0, -vector[0]],
                                [-vector[1], vector[0], 0.0]])
        return Skew_matrix

    # FORWARD KINEMATICS from data recevied by message in mm and deg
    def real_val_encoder_callback(self, msg):

        # getting values from the jointstate messages
        # self.boom_angle_deg =  (msg.position[6] * self.axis_coefficients_encoder_to_geometrical[0] + self.axis_intercepts_encoder_to_geometrical[0])/360*2*math.pi# Axis 3 is the sixth in the jointstate message array 
        # self.boom_extension_mm =  (msg.position[4] * self.axis_coefficients_encoder_to_geometrical[1] + self.axis_intercepts_encoder_to_geometrical[1])# Axis 4 is the fourth in the jointstate message array
        # self.spreader_pitch_deg = (msg.position[5] * self.axis_coefficients_encoder_to_geometrical[2] + self.axis_intercepts_encoder_to_geometrical[2])/360*2*math.pi# Axis 5 is the fifth in the jointstate message array
        # self.spreader_yaw_deg = (msg.position[3] * self.axis_coefficients_encoder_to_geometrical[3] + self.axis_intercepts_encoder_to_geometrical[3])/360*2*math.pi -math.pi/2 # Axis 6 is the third in the jointstate message array  -math.pi/2  #ATTENTION, OUR SPREADER THINKS TO BE ROTATED BY 90 RESPECT TO THE ORIGINAL Z_7 AXIS
        # self.spreader_translation_mm = msg.position[7] * self.axis_coefficients_encoder_to_geometrical[4] + self.axis_intercepts_encoder_to_geometrical[4]# Axis 7 is the seventth in the jointstate message array

        self.encoder_values_received[0] = msg.position[6] # Axis 3 is the sixth in the jointstate message array 
        self.encoder_values_received[1] = msg.position[4] # Axis 4 is the fourth in the jointstate message array
        self.encoder_values_received[2] = msg.position[5] # Axis 5 is the fifth in the jointstate message array
        self.encoder_values_received[3] = msg.position[3] # Axis 6 is the third in the jointstate message array  -math.pi/2  #ATTENTION, OUR SPREADER THINKS TO BE ROTATED BY 90 RESPECT TO THE ORIGINAL Z_7 AXIS
        self.encoder_values_received[4] = msg.position[7] # Axis 7 is the seventth in the jointstate message array

        self.geom_values_received = self.geom_from_encoder_array_transformer(self.encoder_values_received) #transforming the encoder values into the geometrical (mm, deg) values

        self.boom_angle_deg = self.geom_values_received[0]/360*2*math.pi # REMEMBER THAT ROS2 USES RADIANTS  
        self.boom_extension_mm =  self.geom_values_received[1]
        self.spreader_pitch_deg = self.geom_values_received[2]/360*2*math.pi
        self.spreader_yaw_deg = self.geom_values_received[3]/360*2*math.pi -math.pi/2  #ATTENTION, OUR SPREADER THINKS TO BE ROTATED BY 90 RESPECT TO THE ORIGINAL Z_7 AXIS
        self.spreader_translation_mm = self.geom_values_received[4]

        # getting values from the fake messages
        # self.boom_angle_deg = msg.data[0]/360*2*math.pi
        # self.boom_extension_mm =  msg.data[1]
        # self.spreader_pitch_deg = msg.data[2]/360*2*math.pi
        # self.spreader_yaw_deg = msg.data[3]/360*2*math.pi -math.pi/2  #ATTENTION, OUR SPREADER THINKS TO BE ROTATED BY 90 RESPECT TO THE ORIGINAL Z_7 AXIS
        # self.spreader_translation_mm = msg.data[4]


        #considering the O0 stattionary, but able to move

        # publishing O0 position to plotjuggler
        O0_position = [0.0, 0.0, 0.0]  #create a subscriber if the O0 changes position/moves
        O0_position_message = Float32MultiArray()
        O0_position_message.data = [O0_position[0], O0_position[1], O0_position[2]]
        self.plotjuggler_O0_publisher.publish(O0_position_message)

        # # storing O0 position for 3dplot
        # self.x_O0[self.i] = O0_position[0]
        # self.y_O0[self.i] = O0_position[1]
        # self.z_O0[self.i] = O0_position[2]

        #getting first homogeneous transformation
        angle_rot_O0_O1 = self.boom_angle_deg
        print(angle_rot_O0_O1)
        distance_O0_O1 = np.array([[0.0, 0.0, 0.0]]) 
        distance_O0_O1 = distance_O0_O1.T
        R01 = self.get_R_x(angle_rot_O0_O1)
        Hm_O0_O1 = self.Homogen_transf(R01, distance_O0_O1)

        #getting second homogeneous transformation
        angle_rot_O1_O2 = 0.0
        distance_O1_O2 = np.array([[0.0, self.distance_tilt_pin_estendor_y, self.distance_tilt_pin_estendor_z]])
        distance_O1_O2 = distance_O1_O2.T
        R12 = self.get_R_x(angle_rot_O1_O2)
        Hm_O1_O2 = self.Homogen_transf(R12, distance_O1_O2)
        

        R02 = np.matmul(R01, R12)
        Hm_O0_O2 = np.matmul(Hm_O0_O1, Hm_O1_O2)

        # publishing O2 position to plotjuggler
        O2_position = Hm_O0_O2[:,3]
        O2_position_message = Float32MultiArray()
        O2_position_message.data = [O2_position[0], O2_position[1], O2_position[2]]
        self.plotjuggler_O2_publisher.publish(O2_position_message)

        # # storing O2 position for 3dplot
        # self.x_O2[self.i] = O2_position[0]
        # self.y_O2[self.i] = O2_position[1]
        # self.z_O2[self.i] = O2_position[2]

        #getting third homogeneous transformation
        angle_rot_O2_O3 = 0.0
        distance_O2_O3 = np.array([[0.0, self.boom_base_lenght_mm_y + self.boom_extension_mm, self.boom_base_to_pitcher_z]])
        distance_O2_O3 = distance_O2_O3.T
        R23 = self.get_R_x(angle_rot_O2_O3)
        Hm_O2_O3 = self.Homogen_transf(R23, distance_O2_O3)

        R03  = np.matmul(R02, R23)
        Hm_O0_O3 = np.matmul(Hm_O0_O2, Hm_O2_O3)
        
        # publishing O3 position to plotjuggler 
        O3_position = Hm_O0_O3[:, 3]
        O3_position_message = Float32MultiArray()
        O3_position_message.data = [O3_position[0], O3_position[1], O3_position[2]]
        self.plotjuggler_O3_publisher.publish(O3_position_message)

        # # storing O3 position for 3dplot
        # self.x_O3[self.i] = O3_position[0]
        # self.y_O3[self.i] = O3_position[1]
        # self.z_O3[self.i] = O3_position[2]

        #getting fourth homogeneous transformation
        angle_rot_O3_O4 = self.spreader_pitch_deg
        distance_O3_O4 = np.array([[0.0, 0.0, 0.0]])
        distance_O3_O4 = distance_O3_O4.T
        R34 = self.get_R_x(angle_rot_O3_O4)
        Hm_O3_O4 = self.Homogen_transf(R34, distance_O3_O4)

        R04  = np.matmul(R03, R34)
        Hm_O0_O4 = np.matmul(Hm_O0_O3, Hm_O3_O4)

        #getting fifth homogeneous transformation
        angle_rot_O4_O5 = 0.0
        distance_O4_O5 = np.array([[0.0, 0.0, self.spreader_pitch_to_yaw_distance_mm_z]])
        distance_O4_O5 = distance_O4_O5.T
        R45 = self.get_R_x(angle_rot_O4_O5)
        Hm_O4_O5 = self.Homogen_transf(R45, distance_O4_O5)
        
        R05  = np.matmul(R04, R45)
        Hm_O0_O5 = np.matmul(Hm_O0_O4, Hm_O4_O5)

        # publishing O5 position to plotjuggler
        O5_position = Hm_O0_O5[:,3]
        O5_position_message = Float32MultiArray()
        O5_position_message.data = [O5_position[0], O5_position[1], O5_position[2]]
        self.plotjuggler_O5_publisher.publish(O5_position_message)

        # # storing O5 position for 3dplot
        # self.x_O5[self.i] = O5_position[0]
        # self.y_O5[self.i] = O5_position[1]
        # self.z_O5[self.i] = O5_position[2]

        #getting sixth homogeneous transformation
        angle_rot_O5_O6 = self.spreader_yaw_deg
        distance_O5_O6 = np.array([[0.0, 0.0, 0.0]])
        distance_O5_O6 = distance_O5_O6.T
        R56 = self.get_R_z(angle_rot_O5_O6)
        Hm_O5_O6 = self.Homogen_transf(R56, distance_O5_O6)

        R06  = np.matmul(R05, R56)
        Hm_O0_O6 = np.matmul(Hm_O0_O5, Hm_O5_O6)

        #getting seventh homogeneous transformation
        angle_rot_O6_O7 = 0.0
        distance_O6_O7 = np.array([[self.spreader_translation_mm, 0.0, self.yaw_joint_plate_distance_z]])
        distance_O6_O7 = distance_O6_O7.T
        R67 = self.get_R_x(angle_rot_O6_O7)
        Hm_O6_O7 = self.Homogen_transf(R67, distance_O6_O7)

        R07  = np.matmul(R06, R67)
        Hm_O0_O7 = np.matmul(Hm_O0_O6, Hm_O6_O7)

        # PUblishing O7 position to plotjuggler
        O7_position = Hm_O0_O7[:,3]        
        O7_position_message = Float32MultiArray()
        O7_position_message.data = [O7_position[0], O7_position[1], O7_position[2]]
        self.plotjuggler_O7_publisher.publish(O7_position_message)

        # #Saving O7 position for the 3dplot
        # self.x_O7[self.i] = O7_position[0]
        # self.y_O7[self.i] = O7_position[1]
        # self.z_O7[self.i] = O7_position[2]

        #getting eight homogeneous transformation
        angle_rot_O7_O8 = 0.0
        distance_O7_O8 = np.array([[self.distance_innest_to_magnet_mm_x, self.distance_innest_to_magnet_mm_y, self.distance_innest_to_magnet_mm_z]])
        distance_O7_O8 = distance_O7_O8.T
        R78 = self.get_R_x(angle_rot_O7_O8)
        Hm_O7_O8 = self.Homogen_transf(R78, distance_O7_O8)

        R08  = np.matmul(R07, R78)
        Hm_O0_O8 = np.matmul(Hm_O0_O7, Hm_O7_O8)

        # PUblishing O8 position to plotjuggler
        O8_position = Hm_O0_O8[:,3]        
        O8_position_message = Float32MultiArray()
        O8_position_message.data = [O8_position[0], O8_position[1], O8_position[2]]
        self.plotjuggler_O8_publisher.publish(O8_position_message)

        # #Saving O8 position for the 3dplot
        # self.x_O8[self.i] = O8_position[0]
        # self.y_O8[self.i] = O8_position[1]
        # self.z_O8[self.i] = O8_position[2]

        self.i = self.i + 1 

        # #VELOCITY ANALISYS (considering the robot body as stationary)
        # self.w0_0 = msg.data[5]/360*2*math.pi               #rotates around the Y_0 axis            
        # self.w3_3 = msg.data[6]/360*2*math.pi               #rotates around the Y_3 axis
        # self.w5_5 = msg.data[7]/360*2*math.pi               #rotates around the Z_5 axis


        w01_0 = [self.w0_0, 0.0, 0.0]                       #angular velocity between system 0 and 1, reference frame 0
        w12_1 = [0.0, 0.0, 0.0]                             #angular velocity between system 1 and 2, reference frame 1
        w23_2 = [0.0, 0.0, 0.0]                             # etc
        w34_3 = [self.w3_3, 0.0,  0.0]    
        w45_4 = [0.0, 0.0, 0.0]
        w56_5 = [0.0, 0.0, self.w5_5]
        w67_6 = [0.0, 0.0, 0.0]
        w78_7 = [0.0, 0.0, 0.0]                   

        w12_0 = np.matmul(R01, w12_1)
        w23_0 = np.matmul(R02, w23_2)
        w34_0 = np.matmul(R03, w34_3)
        w45_0 = np.matmul(R04, w45_4)
        w56_0 = np.matmul(R05, w56_5)
        w67_0 = np.matmul(R06, w67_6)
        w78_0 = np.matmul(R07, w78_7)

        w02_0 = w01_0 + w12_0
        w03_0 = w02_0 + w23_0 
        w04_0 = w03_0 + w34_0
        w05_0 = w04_0 + w45_0
        w06_0 = w05_0 + w56_0
        w07_0 = w06_0 + w67_0
        w08_0 = w07_0 + w78_0 

        distance_O0_O1_0 = distance_O0_O1
        v_O1 = np.matmul(self.Skew(w01_0), distance_O0_O1_0)

        distance_O1_O2_0 = np.matmul(R02, distance_O1_O2)
        v_O2 = v_O1 + np.matmul(self.Skew(w02_0), distance_O1_O2_0)

        distance_O2_O3_0 = np.matmul(R03, distance_O2_O3)
        v_O3 = v_O2 + np.matmul(self.Skew(w03_0), distance_O2_O3_0)

        distance_O3_O4_0 = np.matmul(R03, distance_O3_O4)
        v_O4 = v_O3 + np.matmul(self.Skew(w04_0), distance_O3_O4_0)

        distance_O4_O5_0 = np.matmul(R04, distance_O4_O5)
        v_O5 = v_O4 + np.matmul(self.Skew(w05_0), distance_O4_O5_0)

        distance_O5_O6_0 = np.matmul(R05, distance_O5_O6)
        v_O6 = v_O5 + np.matmul(self.Skew(w06_0), distance_O5_O6_0)

        distance_O6_O7_0 = np.matmul(R06, distance_O6_O7)
        v_O7 = v_O6 + np.matmul(self.Skew(w07_0), distance_O6_O7_0)

        distance_O7_O8_0 = np.matmul(R07, distance_O7_O8)
        v_O8 = v_O7 + np.matmul(self.Skew(w08_0), distance_O7_O8_0)

        # print(w03_0)
        # print(distance_O0_O3)
        # print(distance_O0_O3_0)        
   
        # PUblishing vel to plotjuggler
        vel = np.concatenate((v_O1, v_O2, v_O3, v_O4, v_O5, v_O6, v_O7, v_O8))   
        vel_flat = vel.flatten().tolist()
        # print(vel) 
        vel_message = Float32MultiArray()
        vel_message.data = vel_flat
        self.plotjuggler_vel_publisher.publish(vel_message) 

    def geom_values_callback(self,msg):

        i = 1
        i = i+1

        # ACCELERATIONS CALCULATIONS TO IMPLEMENT IF I HAVE TIME

    # Creating the plot of all the origin points of our coordinate systems, avoiding the repetivive (coincident) ones
    def plot_3d_data(self):

        # checking if we have all the data
        if self.i == 100:   
            self.i = 0
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            ax.scatter(self.x_O8, self.y_O8, self.z_O8, marker='.', c='magenta', label='O8')
            ax.scatter(self.x_O7, self.y_O7, self.z_O7, marker='.', c='red', label ='O7')
            ax.scatter(self.x_O5, self.y_O5, self.z_O5, marker='.', c='green', label='O5')
            ax.scatter(self.x_O3, self.y_O3, self.z_O3, marker='.', c='blue', label='O3')
            ax.scatter(self.x_O2, self.y_O2, self.z_O2, marker='.', c='magenta', label='O2')
            ax.scatter(self.x_O0, self.y_O0, self.z_O0, marker='.', c='yellow', label='O0')
            ax.legend()

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            # max_range = np.amax([np.amax(self.x_O7), np.amax(self.y_O7), np.amax(self.z_O7)])
            max_range = 1200.0
            ax.set_xlim([-max_range/2, max_range/2])
            ax.set_ylim([-20, max_range])
            ax.set_zlim([-20, max_range])

            #giving the possibility to move in the plot interactively
            ax.mouse_init()
            plt.show()






    
   
   


def main(args=None):
    rclpy.init(args=args)
    joints_model = joint_modeler_node_type()
    rclpy.spin(joints_model)
    joints_model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

