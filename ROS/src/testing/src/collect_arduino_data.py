#!/usr/bin/env python

import os
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from matplotlib import pyplot as plt

wl_wr = []
def fill_array(msg):
    wl = msg.data[0]
    wr = msg.data[1]
    #speed_ang = msg.angular.z
    #speed_lin = msg.linear.x
    #t_zero = int(rospy.Time.now().to_sec())
    wl_wr.append([wl, wr, int(rospy.Time.now().to_sec())])
    rospy.loginfo(wl)

def write_to_csv():
    np.savetxt('data.csv', wl_wr, fmt="%d", delimiter=',')

def init_node():
    rospy.init_node('arduino_data_collector', anonymous=True) #anonymous=True appends a unique ID to the node name
    rospy.Subscriber("arduino_data", Float32MultiArray, fill_array)
    rospy.on_shutdown(write_to_csv)
    rospy.spin()

if __name__ == '__main__':
    init_node()
