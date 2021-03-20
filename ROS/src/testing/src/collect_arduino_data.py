#!/usr/bin/env python

import os
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from matplotlib import pyplot as plt

data = []
def fill_array(msg):
    DIR_L = msg.data[0]
    wl = msg.data[1]
    DIR_R = msg.data[2]
    wr = msg.data[3]
    time = msg.data[4]
    #speed_ang = msg.angular.z
    #speed_lin = msg.linear.x
    #t_zero = int(rospy.Time.now().to_sec())
    data.append([DIR_L, wl, DIR_R, wr, time])
    #rospy.loginfo(wl)

def write_to_csv():
    np.savetxt('data.csv', data, fmt="%d", delimiter=',')

def init_node():
    rospy.init_node('arduino_data_collector', anonymous=True) #anonymous=True appends a unique ID to the node name
    rospy.Subscriber("arduino_data", Float32MultiArray, fill_array)
    rospy.on_shutdown(write_to_csv)
    rospy.spin()

if __name__ == '__main__':
    init_node()
