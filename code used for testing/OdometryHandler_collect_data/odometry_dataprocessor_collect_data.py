#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
import math

class DataProcessor(object):

    def __init__(self, topic_name, r_wheel, wheel_sep):

        self.topic_name = topic_name

        self.r_wheel = r_wheel
        self.wheel_sep = wheel_sep
        self.x_y_theta_t = np.array([0, 0, 0, 0])
        self.vx_vth = [0,0]
        self.collected_data = []

        self._createSubscriber()

    def _createSubscriber(self):
        #wait for message at initialisation before processing data  
        self.arduino_data = None
        while self.arduino_data is None and not rospy.is_shutdown():
            try:
                self.arduino_data = rospy.wait_for_message(self.topic_name, Float32MultiArray, timeout=1.0) #define how long we want to wait for a message
            except:
                rospy.logerr("error in foo") #print error screen after waiting and not receiving a message
        self.processData(self.arduino_data) #calculate twist and pose from odometry information
        self.sub = rospy.Subscriber(self.topic_name, Float32MultiArray, self.processData)

    def processData(self, msg):
        self.wl, self.wr, self.t_ard = self.extractData(msg)

    def extractData(self, msg):
        wl = msg.data[0]*np.pi/180
        wr = msg.data[1]*np.pi/180
        t_ard = msg.data[2]

        return wl, wr, t_ard

    def calcPoseTwist(self, wl, wr, t_ard):
        if ((self.x_y_theta_t == [0, 0, 0, 0]).all()):
            self.x_y_theta_t = np.array([0, 0, 0, t_ard])

        delta_theta = (- wl * self.r_wheel * (t_ard - self.x_y_theta_t[3]) + wr * self.r_wheel * (t_ard - self.x_y_theta_t[3])) / self.wheel_sep
        delta_s = (wl * self.r_wheel * (t_ard - self.x_y_theta_t[3]) + wr * self.r_wheel * (t_ard - self.x_y_theta_t[3])) * 0.5

        x = self.x_y_theta_t[0] + delta_s * np.cos(self.x_y_theta_t[2] * np.pi / 180 + delta_theta * 0.5)               
        y = self.x_y_theta_t[1] + delta_s * np.sin(self.x_y_theta_t[2] * np.pi / 180 + delta_theta * 0.5)
        theta = self.x_y_theta_t[2] + delta_theta * 180 / np.pi
        if theta >= 360:
            theta = theta - (360 * math.floor(theta / 360))
        elif theta <= -360:
            theta = abs(theta)
            theta = -(theta - (360 * math.floor(theta / 360)))

        self.x_y_theta_t = np.array([x, y, theta, t_ard])


        #twist calculation
        if (wl > 0 and wr > 0):
            if (wl > wr):
                self.vx_vth[0] = wr * self.r_wheel
            else:
                self.vx_vth[0] = wl * self.r_wheel
                self.vx_vth[1] = -self.r_wheel * (wl - wr) / (0.5 * self.wheel_sep)
        elif (wl < 0 and wr < 0):
            if (wl < wr):
                self.vx_vth[0] = wr*self.r_wheel
            else:
                self.vx_vth[0] = wl*self.r_wheel
                self.vx_vth[1] = -self.r_wheel * (wl - wr) / (0.5 * self.wheel_sep)
        else:
            self.vx_vth[0] = 0
            self.vx_vth[1] = self.r_wheel * wr / (0.5 * self.wheel_sep)

        return True

    def getBroadcasterInfo(self):
        self.calcPoseTwist(self.wl, self.wr, self.t_ard)

        return self.x_y_theta_t

    def getPublisherInfo(self):
        self.calcPoseTwist(self.wl, self.wr, self.t_ard)

        self.collected_data.append([self.wl, self.wr, rospy.Time.now().to_sec()])

        return self.x_y_theta_t, self.vx_vth

    def setXYTHETA(self):
        self.x_y_theta_t[0] = 0
        self.x_y_theta_t[1] = 0
        self.x_y_theta_t[2] = 0
