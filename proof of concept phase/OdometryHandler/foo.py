#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros

class DataProcessor(object):

	def __init__(self, r_wheel, wheel_sep):

		self.r_wheel = r_wheel
		self.wheel_sep = wheel_sep
		self.x_y_theta_t = np.array([0, 0, 0, 0])
		self.vx_vth = [0,0]

	def extractData(self, msg):

		wl = msg[0]
		wr = msg[1]
		t_ard = msg[2]

		return wl, wr, t_ard
	
	def calcPoseTwist(self, wl, wr, t_ard):
		
		#if (len(self.x_y_theta_t) == 1):
		#	self.x_y_theta_t.append([0, 0, 0, t_ard])

		delta_theta = (- wl * self.r_wheel * (t_ard - self.x_y_theta_t[3]) + wr * self.r_wheel * (t_ard - self.x_y_theta_t[3])) / self.wheel_sep
		delta_s = (wl * self.r_wheel * (t_ard - self.x_y_theta_t[3]) + wr * self.r_wheel * (t_ard - self.x_y_theta_t[3])) * 0.5
		
		x = self.x_y_theta_t[0] + delta_s * np.cos(self.x_y_theta_t[2] * np.pi / 180 + delta_theta * 0.5)		
		y = self.x_y_theta_t[1] + delta_s * np.sin(self.x_y_theta_t[2] * np.pi / 180 + delta_theta * 0.5)
		theta = self.x_y_theta_t[2] + delta_theta * 180 / np.pi  # FIX ME: theta needs to stay between -359 and +359 degrees
		
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

		return self.x_y_theta_t

	def getPublisherInfo(self):

		return self.x_y_theta_t, self.vx_vth


#arduinoDP = DataProcessor(0.025, 0.210)
#wl, wr, t_ard = arduinoDP.extractData([10,10,1])
#arduinoDP.calcPoseTwist(wl, wr, t_ard)
#print(arduinoDP.vx_vth)