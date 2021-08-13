import cv2 as cv
import rospy
import numpy as np
from std_msgs.msg import Int8MultiArray,MultiArrayDimension

windows_height , windows_width , windows_channel = 900 , 900 , 3
windows = np.full((windows_height,windows_width,windows_channel) , 255 , dtype = np.uint8)

#---COLOR MAP---#
GREY    =   (190,190,190)
RED     =   (0,69,255)
WHITE   =   (255,255,255)
BLACK   =   (0,0,0)
ORANGE  =   (15,185,255)

def draw_map(data):

    windows = np.full((windows_height,windows_width,windows_channel) , 255 , dtype = np.uint8)

    # rospy.loginfo(data.data)
    height , width = data.layout.dim[1].size , data.layout.dim[1].stride
    map_matrix = np.reshape( np.mat(data.data) , (height , width))
    pos_x , pos_y = np.linspace(0 , windows_height , num = height + 1 , endpoint=True ,dtype=int) , np.linspace(0 , windows_width , num = width + 1 , endpoint=True , dtype=int)
    for x in range(height):
        for y in range(width):
            plt1 , plt2 = (pos_x[x] , pos_y[y]) , (pos_x[x+1] , pos_y[y + 1])
            # print(plt1,plt2)
            put_rectangle(windows,plt1,plt2,map_matrix[x,y])
    cv.imshow('map',windows)
    if cv.waitKey(1) and 0xFF == ord('Q'):
        rospy.signal_shutdown('User Interrupted!')
    # print(map_matrix)



def put_rectangle(img,plt1,plt2,color):
    if color == -1:
        cv.rectangle(img,plt1,plt2,GREY,-1)
    if color == 0:
        cv.rectangle(img,plt1,plt2,WHITE,-1)
    if color == 1:
        cv.rectangle(img,plt1,plt2,BLACK,-1)
    if color == 2:
        cv.rectangle(img,plt1,plt2,RED,-1)
    if color == 3:
        cv.rectangle(img,plt1,plt2,ORANGE,-1)
    # cv.rectangle(img,plt1,plt2,)
    pass
if __name__ == '__main__':

    # Upper_computer('robot1_3173_nan_ThinkPad',0,0,0)

    rospy.init_node('upper_computer',anonymous=False)
    
    rospy.Subscriber('/map',Int8MultiArray,callback=draw_map,queue_size=1)
    # data_list = [1,2,3,4,5,6,7,8,9]
    rospy.spin()
