#!/usr/bin/env python2


import rospy
from std_msgs.msg import String
from webots_ros.srv import set_float , set_int
from webots_ros.msg import Float64Stamped
from geometry_msgs.msg import Twist , Point , Quaternion
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf
import time

# Wheel Radio (cm)
WHEEL_DIAMETER = 4
# Separation between wheels (cm)
WHEEL_SEPARATION = 5.3
# Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
WHEEL_DISTANCE = 0.053
#Maxspeed in Webots
MAX_SPEED = 6.28
# Wheel circumference (meters).
WHEEL_CIRCUMFERENCE = ((WHEEL_DIAMETER*math.pi)/100.0)
# Distance for each motor step (meters); a complete turn is 1000 steps.
MOT_STEP_DIST = (WHEEL_CIRCUMFERENCE/1000.0)    # 0.000125 meters per step (m/steps)


class E_puck_Test():
    def __init__(self):
        rospy.init_node('Test',anonymous=False)
        
        self.x_pos , self.y_pos , self.theta = 0.0 , 0.0 , 0.0
        self.br = tf.TransformBroadcaster()
        self.leftStepsPrev = 0
        self.rightStepsPrev = 0
        self.leftStepsDiff = 0
        self.rightStepsDiff = 0
        self.deltaSteps = 0
        self.deltaTheta = 0
        self.startTime = time.time()
        self.endTime = time.time()
        #get robot names
        self.robot_names= []

        T1 = rospy.get_time()
        while(rospy.get_time() - T1 < 0.1):
            nameSub = rospy.Subscriber('/model_name',String,callback=self.get_name,queue_size=10)
        nameSub.unregister()

        self.robot_names = list(set(self.robot_names))

        for i , robot_name in enumerate(self.robot_names):
            rospy.wait_for_service(robot_name + '/left_wheel_motor/set_position')
            rospy.wait_for_service(robot_name + '/left_wheel_motor/set_velocity')
            rospy.wait_for_service(robot_name + '/right_wheel_motor/set_position')
            rospy.wait_for_service(robot_name + '/right_wheel_motor/set_velocity')
            
            leftWheelPositionCLient = rospy.ServiceProxy(robot_name + '/left_wheel_motor/set_position' , set_float)
            rightWheelPositionCLient = rospy.ServiceProxy(robot_name + '/right_wheel_motor/set_position' , set_float)
            leftWheelPositionCLient.call(float('inf'))
            rightWheelPositionCLient.call(float('inf'))

            leftWheelSensorCLient = rospy.ServiceProxy(robot_name + '/left_wheel_sensor/enable' , set_int)
            rightWheelSensorCLient = rospy.ServiceProxy(robot_name + '/right_wheel_sensor/enable' , set_int)
            leftWheelSensorCLient.call(1)
            rightWheelSensorCLient.call(1)

            self.leftWheeelVelocityClients = rospy.ServiceProxy(robot_name + '/left_wheel_motor/set_velocity' ,set_float)
            self.rightWheeelVelocityClients = rospy.ServiceProxy(robot_name + '/right_wheel_motor/set_velocity' ,set_float)
            self.rightWheeelVelocityClients.call(0)
            self.leftWheeelVelocityClients.call(0)
            rospy.Subscriber(robot_name + '/vel' , Twist , callback = self.sendVelocity)
            self.motor_pos = [0,0]
            rospy.Subscriber(robot_name + '/left_wheel_sensor/value' , Float64Stamped , callback = self.leftMotorPosition)
            rospy.Subscriber(robot_name + '/right_wheel_sensor/value' , Float64Stamped , callback = self.rightMotorPosition)
            self.odom_publisher = rospy.Publisher(robot_name + '/odom' , Odometry , queue_size = 1)
            print('Successful get service '+ robot_name)
        while not rospy.is_shutdown():
            self.leftStepsDiff = 0.001 * self.motor_pos[0] - self.leftStepsPrev    # Expressed in meters.
            self.rightStepsDiff = 0.001 * self.motor_pos[1] - self.rightStepsPrev  # Expressed in meters.
            #print "left, right steps diff: " + str(self.leftStepsDiff) + ", " + str(self.rightStepsDiff)

            self.deltaTheta = (self.rightStepsDiff - self.leftStepsDiff)/WHEEL_DISTANCE # Expressed in radiant.
            self.deltaSteps = (self.rightStepsDiff + self.leftStepsDiff)/2  # Expressed in meters.
            #print "delta theta, steps: " + str(self.deltaTheta) + ", " + str(self.deltaSteps)

            self.x_pos += self.deltaSteps*math.cos(self.theta + self.deltaTheta/2)  # Expressed in meters.
            self.y_pos += self.deltaSteps*math.sin(self.theta + self.deltaTheta/2)  # Expressed in meters.
            self.theta += self.deltaTheta   # Expressed in radiant.
            #print "x, y, theta: " + str(self.x_pos) + ", " + str(self.y_pos) + ", " + str(self.theta)

            self.leftStepsPrev = 0.001 * self.motor_pos[0]  # Expressed in meters.
            self.rightStepsPrev = 0.001 * self.motor_pos[1]    # Expressed in meters.

            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "/base_link"
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
            pos = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
            ori = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
            self.br.sendTransform(pos, ori, odom_msg.header.stamp, odom_msg.child_frame_id, odom_msg.header.frame_id)

    def get_name(self,data):
        if data.data not in self.robot_names:

            robot_name = data.data.split('_')[0]
            if robot_name != 'master':
                self.robot_names.append(data.data)

    def sendVelocity(self,data):
        linear , angular = data.linear.x , data.angular.z
        # Kinematic model for differential robot.
        wl = (linear - (WHEEL_SEPARATION / 2.) * angular) / WHEEL_DIAMETER
        wr = (linear + (WHEEL_SEPARATION / 2.) * angular) / WHEEL_DIAMETER
        # At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
        left_vel ,right_vel = np.clip(wl * 10.,-MAX_SPEED , MAX_SPEED) , np.clip(wr * 10 , - MAX_SPEED , MAX_SPEED)
        self.leftWheeelVelocityClients.call(left_vel)
        self.rightWheeelVelocityClients.call(right_vel)

    def leftMotorPosition(self,data):
        self.motor_pos[0] = data.data

    def rightMotorPosition(self,data):
        self.motor_pos[1] = data.data

if __name__ == '__main__':
    robot = E_puck_Test()
    rospy.spin()