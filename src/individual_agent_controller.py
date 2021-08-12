#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int8MultiArray
from webots_ros.srv import set_float , set_int
from webots_ros.msg import Float64Stamped
from geometry_msgs.msg import Twist , Point , Quaternion
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf
import message_filters
import time

# Wheel Radio (cm)
WHEEL_DIAMETER = 4
# Separation between wheels (cm)
WHEEL_SEPARATION = 5.2
# Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
WHEEL_DISTANCE = 0.052
#Maxspeed in Webots
MAX_SPEED = 6.28
# Wheel circumference (meters).
WHEEL_CIRCUMFERENCE = ((WHEEL_DIAMETER*math.pi)/100.0)
# Distance for each motor step (meters); a complete turn is 1000 steps.
MOT_STEP_DIST = (WHEEL_CIRCUMFERENCE/1000.0)    # 0.000125 meters per step (m/steps)
# Chess Board Size(m)
BOARD_SIZE = 0.01

class E_puck_Test():
    def __init__(self,robot_name,init_x,init_y,init_theta):
        self.robot_name = robot_name
        rospy.init_node(robot_name + '_Core_Node',anonymous=False)
        self.linear , self.angular = 0.0 , 0.0
        self.leftMotor , self.rightMotor = self.Motor_Init(self.robot_name)
        self.Encoder_Init(robot_name)
        #Init Odometry 
        self.x_pos , self.y_pos , self.theta = init_x , init_y , init_theta
        self.br = tf.TransformBroadcaster()
        self.leftStepsPrev = 0
        self.rightStepsPrev = 0
        self.leftStepsDiff = 0
        self.rightStepsDiff = 0
        self.deltaSteps = 0
        self.deltaTheta = 0
        self.startTime = time.time()
        self.endTime = time.time()

        #Start Velocity Topic
        rospy.Subscriber(robot_name + '/vel' , Twist , callback = self.Velocity_callback)
        #Get Encoder Value
        self.motor_pos = [0,0]

        #Odometry Publisher
        self.odom_publisher = rospy.Publisher(robot_name + '/odom' , Odometry,queue_size=1)
        self.rate = rospy.Rate(100)

        #Init Proximity
        self.Proximity_Init()

        #Init Map
        self.map_msg = Int8MultiArray(layout = Point())
        self.map_msg.layout.x  , self.map_msg.layout.y = init_x , init_y
        self.last_position = Point(init_x , init_y , init_theta)
        # print(np.ones((3,3),dtype=int))
        self.map_msg.data = -1 * np.mat((np.ones((3,3),dtype=int)))
        self.map_msg.data[1,1] = 2
        print(self.map_msg.data)
        self.mapPublisher = rospy.Publisher(self.robot_name + '/map' , Int8MultiArray , queue_size = 1)
        while not rospy.is_shutdown():
            self.get_Odometry()

        rospy.loginfo('---Start ' + self.robot_name + '_ROS_Node Susseccfully')

    def get_Odometry(self):
        self.leftStepsDiff = self.motor_pos[0] * (0.01 * WHEEL_DIAMETER / 2) - self.leftStepsPrev    # Expressed in meters.
        self.rightStepsDiff = self.motor_pos[1] * (0.01 * WHEEL_DIAMETER / 2)- self.rightStepsPrev  # Expressed in meters.
        #print "left, right steps diff: " + str(self.leftStepsDiff) + ", " + str(self.rightStepsDiff)

        self.deltaTheta = (self.rightStepsDiff - self.leftStepsDiff)/WHEEL_DISTANCE # Expressed in radiant.
        self.deltaSteps = (self.rightStepsDiff + self.leftStepsDiff)/2  # Expressed in meters.
        # print('deltaTheta:' + str(self.deltaTheta) + 'deltaSteps:' + str(self.deltaSteps))

        self.x_pos += self.deltaSteps * math.cos(self.theta + self.deltaTheta/2)  # Expressed in meters.
        self.y_pos += self.deltaSteps * math.sin(self.theta + self.deltaTheta/2)  # Expressed in meters.
        self.theta += self.deltaTheta   # Expressed in radiant.
        #print "x, y, theta: " + str(self.x_pos) + ", " + str(self.y_pos) + ", " + str(self.theta)

        self.leftStepsPrev = (0.01 * WHEEL_DIAMETER / 2) * self.motor_pos[0]  # Expressed in meters.
        self.rightStepsPrev = (0.01 * WHEEL_DIAMETER / 2) * self.motor_pos[1]    # Expressed in meters.

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = self.robot_name + "/base_link"
        odom_msg.pose.pose.position = Point(self.x_pos, self.y_pos, 0)
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*q)
        self.endTime = time.time()
        odom_msg.twist.twist.linear.x = self.deltaSteps / (self.endTime-self.startTime) # self.deltaSteps is the linear distance covered in meters from the last update (delta distance);
                                                                                        # the time from the last update is measured in seconds thus to get m/s we multiply them.
        odom_msg.twist.twist.angular.z = self.deltaTheta / (self.endTime-self.startTime)    # self.deltaTheta is the angular distance covered in radiant from the last update (delta angle);
                                                                                            # the time from the last update is measured in seconds thus to get rad/s we multiply them.
        #print "time elapsed = " + str(self.endTime-self.startTime) + " seconds"
        self.startTime = self.endTime

        self.odom_publisher.publish(odom_msg)
        self.rate.sleep()
        pos = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
        ori = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.br.sendTransform(pos, ori, odom_msg.header.stamp, odom_msg.child_frame_id, odom_msg.header.frame_id)
 
    def Motor_Init(self,robot_name):
        #Init Motor_position && Motor_velocity
        rospy.wait_for_service(robot_name + '/left_wheel_motor/set_position')
        rospy.wait_for_service(robot_name + '/left_wheel_motor/set_velocity')
        rospy.wait_for_service(robot_name + '/right_wheel_motor/set_position')
        rospy.wait_for_service(robot_name + '/right_wheel_motor/set_velocity')
        
        leftWheelPositionCLient = rospy.ServiceProxy(robot_name + '/left_wheel_motor/set_position' , set_float)
        rightWheelPositionCLient = rospy.ServiceProxy(robot_name + '/right_wheel_motor/set_position' , set_float)
        leftWheelPositionCLient.call(float('inf'))
        rightWheelPositionCLient.call(float('inf'))

        leftWheeelVelocityClients = rospy.ServiceProxy(robot_name + '/left_wheel_motor/set_velocity' ,set_float)
        rightWheeelVelocityClients = rospy.ServiceProxy(robot_name + '/right_wheel_motor/set_velocity' ,set_float)
        rightWheeelVelocityClients.call(0)
        leftWheeelVelocityClients.call(0)

        leftWheelPositionCLient.close()
        rightWheelPositionCLient.close()

        rospy.loginfo('---Init Motor Successful---')
        return leftWheeelVelocityClients , rightWheeelVelocityClients

    def Encoder_Init(self,robot_name):
        #Init Motor_Encoder
        leftWheelSensorCLient = rospy.ServiceProxy(robot_name + '/left_wheel_sensor/enable' , set_int)
        rightWheelSensorCLient = rospy.ServiceProxy(robot_name + '/right_wheel_sensor/enable' , set_int)
        leftWheelSensorCLient.call(1)
        rightWheelSensorCLient.call(1)

        leftWheelSensorCLient.close()
        rightWheelSensorCLient.close()

        leftSubscriber = message_filters.Subscriber(robot_name + '/left_wheel_sensor/value' , Float64Stamped)
        rightSubscriber = message_filters.Subscriber(robot_name + '/right_wheel_sensor/value' , Float64Stamped)
        encoderSubscriber = message_filters.ApproximateTimeSynchronizer([leftSubscriber,rightSubscriber] , 10 , 0.1 ,allow_headerless = True)
        encoderSubscriber.registerCallback(self.encoder_callback)
        rospy.loginfo('---Init Encoder Successful---')

    def Proximity_Init(self):
        #Init Proximity
        psSubscribers = []
        for i in range(8):
            proximityEnableCLient = rospy.ServiceProxy(self.robot_name + '/ps' + str(i) + '/enable' , set_int)
            proximityEnableCLient.call(1)
            proximityEnableCLient.close()

            psSubscribers.append(message_filters.Subscriber(self.robot_name + '/ps' + str(i) + '/value' , Range))

        MapSubscriber = message_filters.ApproximateTimeSynchronizer(psSubscribers , 10 , 0.1 , allow_headerless = True)
        MapSubscriber.registerCallback(self.Generate_Map)
        rospy.loginfo('---Init Proximity Successful---')

    def Generate_Map(self , *map_Data):
        #---Definition of map---#
        # -1    ---     not detection yet       # 
        # 0     ---     no bondary              #        
        # 1     ---     bondary                 #
        # 2     ---     e-puck current position #

        #---Position Update---#
        height , width = np.shape(self.map_msg.data)
        current_position = np.where(self.map_msg.data == 2)
        delta_move_x , delta_move_y = (self.x_pos - self.last_position.x)/BOARD_SIZE , (self.y_pos - self.last_position.y)/BOARD_SIZE
        next_position = [int(current_position[0] + int(delta_move_x)) , int(current_position[1] + int(delta_move_y))]
        
        next_position[0] , next_position[1] = np.clip(next_position[0] , 0 , height) , np.clip(next_position[1], 0 ,width)
        print(next_position)
        #Updata new position
        if next_position[0] != current_position[0] or next_position[1] != current_position[1]:
            self.map_msg.data[next_position[0] , next_position[1]] = 2
            self.map_msg.data[current_position[0] , current_position[1]] = 0
            self.last_position.x , self.last_position.y = self.x_pos , self.y_pos
            #Enlarge map
            if next_position[0] == 0:
                self.map_msg.data = np.r_[np.mat(-1 * np.ones((1,width) , dtype = int)) , self.map_msg.data]
            if next_position[0] == height - 1:
                self.map_msg.data = np.r_[self.map_msg.data , np.mat(-1 * np.ones((1 , width) , dtype = int))]
            if next_position[1] == 0: 
                self.map_msg.data = np.c_[np.mat(-1 * np.ones((height + 1 , 1) , dtype = int)) , self.map_msg.data]
            if next_position[1] == width:
                self.map_msg.data = np.c_[self.map_msg.data , np.mat(-1 * np.ones((height + 1 , 1) , dtype = int))]
        print(self.map_msg.data)
        #If Sensor Data Number is not enough , throwout ERROR
        if(len(map_Data) != 8):
            rospy.logerr_once('Data not enough to generate map')
        for i , item in enumerate(map_Data):
            if i == 8:
                continue
            pass

    def Velocity_callback(self,data):
        if data.linear.x == self.linear and data.angular.z == self.angular:
            return None
        #Control Pi-puck Move && Spin
        self.linear , self.angular = data.linear.x , data.angular.z
        # Kinematic model for differential robot.
        wl = (self.linear - (WHEEL_SEPARATION / 2.) * self.angular) / WHEEL_DIAMETER
        wr = (self.linear + (WHEEL_SEPARATION / 2.) * self.angular) / WHEEL_DIAMETER
        # At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
        left_vel ,right_vel = np.clip(wl * 10.,-MAX_SPEED , MAX_SPEED) , np.clip(wr * 10 , - MAX_SPEED , MAX_SPEED)
        self.leftMotor.call(left_vel)
        self.rightMotor.call(right_vel)
        
    def encoder_callback(self,leftMotor,rightMotor):
        # print(leftMotor.data,rightMotor.data)
        self.motor_pos = [leftMotor.data , rightMotor.data]

if __name__ == '__main__':
    robot_name = 'robot1_3173_nan_ThinkPad'
    init_x , init_y , init_theta = 0.0 , 0.0 , 0.0
    robot = E_puck_Test(robot_name,init_x,init_y,init_theta)
    rospy.spin()