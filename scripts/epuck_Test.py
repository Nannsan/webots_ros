#!/usr/bin/env python3


from pickle import FLOAT
import rospy
from std_msgs.msg import String,Float64
from geometry_msgs.msg import Twist
import numpy as np

class E_puck_Test():
    def __init__(self):
        rospy.init_node('Test',anonymous=False)
        
        #get robot names
        self.robot_names= []
        T1 = rospy.get_time()
        while(rospy.get_time() - T1 < 0.5):
            rospy.Subscriber('/model_name',String,callback=self.get_name)

        #start robot Velocity controller topic
        for i , robot_name in enumerate(self.robot_names):
            print(i , robot_name)
            rospy.wait_for_service(robot_name + '/left_wheel_motor/set_position')
            rospy.wait_for_service(robot_name + '/left_wheel_motor/set_velocity')
            rospy.wait_for_service(robot_name + '/right_wheel_motor/set_position')
            rospy.wait_for_service(robot_name + '/right_wheel_motor/set_velocity')
            
            leftWheelPositionCLient = rospy.ServiceProxy(robot_name + '/left_wheel_motor/set_position' , Float64)
            rightWheelPositionCLient = rospy.ServiceProxy(robot_name + '/right_wheel_motor/set_position' , Float64)
            leftWheelPositionCLient.call(float('inf'))
            rightWheelPositionCLient.call(float('inf'))

            print('Successful get service '+ robot_name)
            # pass
    def get_name(self,data):
        if data.data not in self.robot_names:
            self.robot_names.append(data.data)




if __name__ == '__main__':
    robot = E_puck_Test()
