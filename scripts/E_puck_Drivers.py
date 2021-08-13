#!/usr/bin/env python2

'''
E_puck Driver used on simulation in Webots
'''
from logging import DEBUG
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int8MultiArray , MultiArrayDimension
from webots_ros.srv import set_float , set_int
from webots_ros.msg import Float64Stamped
from geometry_msgs.msg import Twist , Point , Quaternion ,Pose
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf
import message_filters
import time

# Wheel Diameter (cm)
WHEEL_DIAMETER = 4
# Wheel Radius (m)
WHEEL_RADIUS = 0.02
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
# Option on Debug Output
DEBUG_FLAG = True
# Option on MapPublisher
DRAW_MAP = True
# Available sensors
sensors = ['motors' , 'proximity' , 'encoder' , 'odometry' , 'map']

class msg_param(object):
    # Message Params
    def __init__(self,robot_name,msg_name,*args):
        self.robot_name = robot_name
        if msg_name == 'encoder_msg':
            self.encoder_msg_param()
        if msg_name == 'odom_msg':
            self.odom_msg_param(args)
        if msg_name == 'map_msg':
            self.map_msg_param(args)

    def encoder_msg_param(self):
        self.motor_pos = [0,0]

    def odom_msg_param(self,args):
        # args: arg[0] Pose
        self.pose = args[0]
        self.br = tf.TransformBroadcaster()
        self.leftStepsPrev = 0
        self.rightStepsPrev = 0
        self.leftStepsDiff = 0
        self.rightStepsDiff = 0
        self.deltaSteps = 0
        self.deltaTheta = 0
        self.startTime = time.time()
        self.endTime = time.time()

        self.odom_msg = Odometry()
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = self.robot_name + "/base_link"
        self.odom_msg.pose.pose.position = Point(self.pose.position.x, self.pose.position.y, 0)
        q = tf.transformations.quaternion_from_euler(0, 0, self.pose.orientation.z)
        self.odom_msg.pose.pose.orientation = Quaternion(*q)
        self.endTime = time.time()
        self.odom_msg.twist.twist.linear.x = self.deltaSteps / (self.endTime-self.startTime) # self.deltaSteps is the linear distance covered in meters from the last update (delta distance);
                                                                                        # the time from the last update is measured in seconds thus to get m/s we multiply them.
        self.odom_msg.twist.twist.angular.z = self.deltaTheta / (self.endTime-self.startTime)    # self.deltaTheta is the angular distance covered in radiant from the last update (delta angle);
                                                                                            # the time from the last update is measured in seconds thus to get rad/s we multiply them.

        self.odom_publisher = rospy.Publisher(self.robot_name + '/odom' , Odometry,queue_size=1)
    
    def map_msg_param(self,args):
        #args: arg[0] ---  (init_x ,init_y init_theta)
        init_x , init_y , init_theta= args[0][0] , args[0][1] , args[0][2]
        position_dimension , self.Map_dimension = MultiArrayDimension() ,MultiArrayDimension()
        self.map_msg = Int8MultiArray()
        position_dimension.label , position_dimension.size , position_dimension.stride = 'Init_pos' , init_x , init_y
        self.map_msg.layout.dim.append(position_dimension)
        self.last_position = Point(init_x , init_y , init_theta)
        self.height , self.width = 3 , 3
        map_matrix = -1 * np.mat((np.ones((self.height,self.width),dtype=int)))
        self.current_position = (1,1)
        map_matrix[self.current_position[0],self.current_position[1]] = 2
        self.Map_dimension.label , self.Map_dimension.size , self.Map_dimension.stride = 'Map' , self.height , self.width
        
        self.map_msg.layout.dim.append(self.Map_dimension)
        self.map_msg.data = Convert_MapData(map_matrix)
        if DRAW_MAP: self.mapPublisher = rospy.Publisher('/map' , Int8MultiArray , queue_size = 1)

class E_puck_Drivers(msg_param):
    def __init__(self,robot_name,pos_list):
        self.Greetings(robot_name , pos_list)
        self.Sensors_Init(sensors)

        self.rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.Sensors_Update(sensors)
            self.rate.sleep()


    def Greetings(self , robot_name , pos_list, **kwd):
        self.robot_name = robot_name
        self.pose , self.vel = Pose() , Twist()
        self.vel.linear.x , self.vel.angular.z = 0.0 ,0.0
        self.pose.position.x , self.pose.position.y , self.pose.orientation.z = pos_list[0] , pos_list[1] , pos_list[2]
        if DEBUG_FLAG == False:
            log_level = rospy.WARN
        rospy.init_node(robot_name + '_Core_Node' , anonymous = False , log_level=log_level)
        rospy.logwarn_once('%s is starting working!' , robot_name)
        pass

    def Sensors_Init(self , sensors):
        if 'motors' in sensors:
            leftMOtor , rightMotor = self.Motor_Init()
            rospy.Subscriber(self.robot_name + '/vel' , Twist , callback = self.Velocity_callback , callback_args = [leftMOtor , rightMotor])
            rospy.loginfo_once('--- Motor Ready! ---')
        if 'proximity' in sensors:
            self.Proximity_Init()
            rospy.loginfo_once('--- Proximity Ready! ---')
        if 'encoder' in sensors:
            self.encoder_params = msg_param(self.robot_name,'encoder_msg')
            self.Encoder_Init()
            rospy.loginfo_once('--- Encoder Ready! ---')
            if 'odometry' in sensors:
                self.odom_params = msg_param(self.robot_name,'odom_msg',self.pose)
                rospy.loginfo_once('--- Odometry Ready! ---')
                if 'map' in sensors:
                    self.map_params = msg_param(self.robot_name,'map_msg',(self.pose.position.x,self.pose.position.y,self.pose.orientation.z))
                    MapSubscriber = message_filters.ApproximateTimeSynchronizer(self.psSubscribers , 10 , 0.1 , allow_headerless = True)
                    MapSubscriber.registerCallback(self.Generate_Map)
                    rospy.loginfo_once('--- Map Ready! ---')

    def Sensors_Update(self , sensors):
        if 'odometry' in sensors:
            self.get_Odometry()
            if 'map' in sensors:
                if DRAW_MAP: self.map_params.mapPublisher.publish(self.map_params.map_msg)

    def get_Odometry(self):
        self.odom_params.leftStepsDiff = self.encoder_params.motor_pos[0] * WHEEL_RADIUS - self.odom_params.leftStepsPrev    # Expressed in meters.
        self.odom_params.rightStepsDiff = self.encoder_params.motor_pos[1] * WHEEL_RADIUS- self.odom_params.rightStepsPrev  # Expressed in meters.

        self.odom_params.deltaTheta = (self.odom_params.rightStepsDiff - self.odom_params.leftStepsDiff)/WHEEL_DISTANCE # Expressed in radiant.
        self.odom_params.deltaSteps = (self.odom_params.rightStepsDiff + self.odom_params.leftStepsDiff)/2  # Expressed in meters.
    
        self.pose.position.x += self.odom_params.deltaSteps * math.cos(self.pose.orientation.z + self.odom_params.deltaTheta/2)  # Expressed in meters.
        self.pose.position.y += self.odom_params.deltaSteps * math.sin(self.pose.orientation.z + self.odom_params.deltaTheta/2)  # Expressed in meters.
        self.pose.orientation.z += self.odom_params.deltaTheta   # Expressed in radiant.

        self.odom_params.leftStepsPrev  =   WHEEL_RADIUS *  self.encoder_params.motor_pos[0]    # Expressed in meters.
        self.odom_params.rightStepsPrev =   WHEEL_RADIUS *  self.encoder_params.motor_pos[1]    # Expressed in meters.

        self.odom_params.odom_msg.pose.pose.position = Point(self.pose.position.x, self.pose.position.y, 0)
        q = tf.transformations.quaternion_from_euler(0, 0, self.pose.orientation.z)
        self.odom_params.odom_msg.pose.pose.orientation = Quaternion(*q)
        self.odom_params.endTime = time.time()
        self.odom_params.odom_msg.twist.twist.linear.x = self.odom_params.deltaSteps / (self.odom_params.endTime-self.odom_params.startTime) # self.deltaSteps is the linear distance covered in meters from the last update (delta distance);
                                                                                        # the time from the last update is measured in seconds thus to get m/s we multiply them.
        self.odom_params.odom_msg.twist.twist.angular.z = self.odom_params.deltaTheta / (self.odom_params.endTime-self.odom_params.startTime)    # self.deltaTheta is the angular distance covered in radiant from the last update (delta angle);
                                                                                            # the time from the last update is measured in seconds thus to get rad/s we multiply them.
 
        self.odom_params.startTime = self.odom_params.endTime

        self.odom_params.odom_publisher.publish(self.odom_params.odom_msg)
        
        pos = (self.odom_params.odom_msg.pose.pose.position.x, self.odom_params.odom_msg.pose.pose.position.y, self.odom_params.odom_msg.pose.pose.position.z)
        ori = (self.odom_params.odom_msg.pose.pose.orientation.x, self.odom_params.odom_msg.pose.pose.orientation.y, self.odom_params.odom_msg.pose.pose.orientation.z, self.odom_params.odom_msg.pose.pose.orientation.w)
        self.odom_params.br.sendTransform(pos, ori, self.odom_params.odom_msg.header.stamp, self.odom_params.odom_msg.child_frame_id, self.odom_params.odom_msg.header.frame_id)

    def Motor_Init(self):
        #Init Motor_position && Motor_velocity
        rospy.wait_for_service(self.robot_name + '/left_wheel_motor/set_position')
        rospy.wait_for_service(self.robot_name + '/left_wheel_motor/set_velocity')
        rospy.wait_for_service(self.robot_name + '/right_wheel_motor/set_position')
        rospy.wait_for_service(self.robot_name + '/right_wheel_motor/set_velocity')
        
        leftWheelPositionCLient = rospy.ServiceProxy(self.robot_name + '/left_wheel_motor/set_position' , set_float)
        rightWheelPositionCLient = rospy.ServiceProxy(self.robot_name + '/right_wheel_motor/set_position' , set_float)
        leftWheelPositionCLient.call(float('inf'))
        rightWheelPositionCLient.call(float('inf'))

        leftWheeelVelocityClients = rospy.ServiceProxy(self.robot_name + '/left_wheel_motor/set_velocity' ,set_float)
        rightWheeelVelocityClients = rospy.ServiceProxy(self.robot_name + '/right_wheel_motor/set_velocity' ,set_float)
        rightWheeelVelocityClients.call(0)
        leftWheeelVelocityClients.call(0)

        leftWheelPositionCLient.close()
        rightWheelPositionCLient.close()
        return leftWheeelVelocityClients , rightWheeelVelocityClients

    def Encoder_Init(self):
        #Init Motor_Encoder
        leftWheelSensorCLient = rospy.ServiceProxy(self.robot_name + '/left_wheel_sensor/enable' , set_int)
        rightWheelSensorCLient = rospy.ServiceProxy(self.robot_name + '/right_wheel_sensor/enable' , set_int)
        leftWheelSensorCLient.call(1)
        rightWheelSensorCLient.call(1)

        leftWheelSensorCLient.close()
        rightWheelSensorCLient.close()

        leftSubscriber = message_filters.Subscriber(self.robot_name + '/left_wheel_sensor/value' , Float64Stamped)
        rightSubscriber = message_filters.Subscriber(self.robot_name + '/right_wheel_sensor/value' , Float64Stamped)
        encoderSubscriber = message_filters.ApproximateTimeSynchronizer([leftSubscriber,rightSubscriber] , 10 , 0.1 ,allow_headerless = True)
        encoderSubscriber.registerCallback(self.encoder_callback)

    def Proximity_Init(self):
        #Init Proximity
        self.psSubscribers = []
        for i in range(8):
            proximityEnableCLient = rospy.ServiceProxy(self.robot_name + '/ps' + str(i) + '/enable' , set_int)
            proximityEnableCLient.call(1)
            proximityEnableCLient.close()

            self.psSubscribers.append(message_filters.Subscriber(self.robot_name + '/ps' + str(i) + '/value' , Range))

    def Generate_Map(self , *map_Data):
        map_matrix = np.reshape(np.mat(self.map_params.map_msg.data) , (self.map_params.height , self.map_params.width))
        #           Definition of map           #
        # -1    ---     not detected yet        # 
        # 0     ---     no bondary              #        
        # 1     ---     bondary                 #
        # 2     ---     e-puck current position #

        #---Position Update---#
        current_position , last_position = self.map_params.current_position , self.map_params.last_position
        delta_move_x , delta_move_y = -(self.pose.position.x - last_position.x)/BOARD_SIZE , -(self.pose.position.y - last_position.y)/BOARD_SIZE
        next_position = [int(current_position[0] + int(delta_move_x)) , int(current_position[1] + int(delta_move_y))]

        #Updata new position
        if next_position[0] != current_position[0] or next_position[1] != current_position[1]:
            map_matrix[next_position[0] , next_position[1]] = 2
            map_matrix[current_position[0] , current_position[1]] = 0
            self.map_params.last_position.x , self.map_params.last_position.y = self.pose.position.x , self.pose.position.y
            #Enlarge map
            if next_position[0] == 0:
                map_matrix = np.r_[np.mat(np.full( (1 , np.shape(map_matrix)[1]) , -1 , dtype = int)) , map_matrix]
            if next_position[0] == np.shape(map_matrix)[0] - 1:
                map_matrix = np.r_[map_matrix , np.mat( np.full ((1 , np.shape(map_matrix)[1]) , -1 , dtype = int))]
            if next_position[1] == 0: 
                map_matrix = np.c_[np.mat( np.full( (np.shape(map_matrix)[0] , 1) , -1 , dtype = int)) , map_matrix]
            if next_position[1] == np.shape(map_matrix)[1] - 1:
                map_matrix = np.c_[map_matrix , np.mat( np.full ( (np.shape(map_matrix)[0] , 1) , -1 , dtype = int))]  
            self.map_params.height , self.map_params.width = np.shape(map_matrix)
            self.map_params.Map_dimension.label , self.map_params.Map_dimension.size , self.map_params.Map_dimension.stride = 'Map' , self.map_params.height , self.map_params.width
            self.map_params.current_position = np.where(map_matrix == 2)

        #---Proximity Sequence---#
        #|--------------------- |#
        #|  0   |   1   |   2   |#
        #|----------------------|#
        #|  7   |       |   3   |#
        #|----------------------|#
        #|  6   |   5   |   4   |#
        #|----------------------|#  
        pos_degree = self.pose.orientation.z * 180 / math.pi
        psSequence = list_move_right([0,1,2,7,3,6,5,4] , int(pos_degree / 45))
        psSequence.insert(4,8)

        for i , item in enumerate(map_Data):
            row , line = psSequence.index(i) % 3 - 1 , psSequence.index(i) / 3 - 1
            if item.range > 1000:
                rospy.loginfo('ps' + str(i) + 'get obstacle')
                map_matrix[self.map_params.current_position[0] + row , self.map_params.current_position[1] + line] = 1
            else:
                map_matrix[self.map_params.current_position[0] + row , self.map_params.current_position[1] + line] = 0
        self.map_params.map_msg.data = Convert_MapData(map_matrix)

    def Velocity_callback(self,data,motor):
        if data.linear.x == self.vel.linear.x and data.angular.z == self.vel.angular.z:
            return None
        #Control Pi-puck Move && Spin
        self.vel.linear.x , self.vel.angular.z = data.linear.x , data.angular.z
        # Kinematic model for differential robot.
        wl = (self.vel.linear.x - (WHEEL_SEPARATION / 2.) * self.vel.angular.z) / WHEEL_DIAMETER
        wr = (self.vel.linear.x + (WHEEL_SEPARATION / 2.) * self.vel.angular.z) / WHEEL_DIAMETER
        # At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
        left_vel ,right_vel = np.clip(wl * 10.,-MAX_SPEED , MAX_SPEED) , np.clip(wr * 10 , - MAX_SPEED , MAX_SPEED)
        motor[0].call(left_vel)
        motor[1].call(right_vel)
        
    def encoder_callback(self,leftMotor,rightMotor):
        # print(leftMotor.data,rightMotor.data)
        self.encoder_params.motor_pos = [leftMotor.data , rightMotor.data]

def Convert_MapData(data):
    #---Convert matrix data to list data---#
    data = data.flatten()
    data = data.tolist()
    return data[0]

def list_move_right(list , k):
    #---Move list---#
    for _ in range(k):
        list.insert(0,list.pop())
    return list

if __name__ == '__main__':
    #---Robot name changes when restarting webots---#
    robot_name = 'robot1_3477_nan_ThinkPad'
    init_x , init_y , init_theta = 0.0 , 0.0 , 0.0
    robot = E_puck_Drivers(robot_name,(init_x,init_y,init_theta))
    rospy.spin()