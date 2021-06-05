#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

x_y_theta_t = [[0, 0, 0, 0]]
r_wheel = 0.025
wheel_sep = 0.210


def calc_pos(msg):
    if (len(x_y_theta_t) == 1):
        x_y_theta_t.append([0, 0, 0, msg.data[2]])
    wl = msg.data[0]*np.pi/180
    wr = msg.data[1]*np.pi/180
    t_ard = msg.data[2]
   
    delta_theta = (-wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) / wheel_sep
    delta_s = (wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) * 0.5
    x = x_y_theta_t[-1][0] + delta_s * np.cos(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    y = x_y_theta_t[-1][1] + delta_s * np.sin(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    theta = x_y_theta_t[-1][2] + delta_theta*180/np.pi #FIX ME: theta needs to stay between -359 and +359 degrees
    
    x_y_theta_t.append([x, y, theta, t_ard])



def write_to_csv():
    np.savetxt('data.csv', x_y_theta_t, fmt="%1.3f", delimiter=',')


def init_node():
    rospy.init_node('arduino_data_collector', anonymous=True)  # anonymous=True appends a unique ID to the node name
    rospy.Subscriber("arduino_data", Float32MultiArray, calc_pos)
    rospy.on_shutdown(write_to_csv)
    rospy.spin()


if __name__ == '__main__':
    init_node()
