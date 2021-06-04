#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
import tf_conversions
import tf2_ros

from foo import DataProcessor
from bar import Publisher

class OdometryHandler(object):

	def __init__(self):
		self.arduino_data_processor = DataProcessor(0.025, 0.210)
		self.pub = Publisher("odom", "base_link")
		self.firstLoc = True

	
	def simulate(self, sim_data):
		wl, wr, t_ard = self.arduino_data_processor.extractData(sim_data)
		self.arduino_data_processor.calcPoseTwist(wl, wr, t_ard)
		x_y_theta_t, vx_vth = self.arduino_data_processor.getPublisherInfo()
		msg = self.pub.createNavMsg(10, x_y_theta_t, vx_vth)

		if self.firstLoc:
			plt.plot(msg.pose.pose.position.x, msg.pose.pose.position.y, 'r.')
			self.firstLoc = False
		else:
			plt.plot(msg.pose.pose.position.x, msg.pose.pose.position.y, 'b.')        
			#print(self.pub.createNavMsg(1200, x_y_theta_t, vx_vth))


r_wheel = 0.025
wheel_sep = 0.210
'''
sim_vel = [[10, 10, 1],
	[10, 10, 2],
			 [10, 10, 3],
			 [10, 10, 4],
			 [-(wheel_sep*0.5*np.pi)/(2*r_wheel), (wheel_sep*0.5*np.pi)/(2*r_wheel), 5],
			 [10, 10, 6],
			 [10, 10, 7],
			 [10, 10, 8],
			 [10, 10, 9],
			 [-(wheel_sep*0.5*np.pi)/(2*r_wheel), (wheel_sep*0.5*np.pi)/(2*r_wheel), 10],
			 [10, 10, 11],
			 [10, 10, 12],
			 [10, 10, 13],
			 [10, 10, 14]]
'''

fig = plt.figure()
odomhandler = OdometryHandler()

import csv

with open('square_20hz.csv') as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	for row in csv_reader:
		sim_data = [float(row[0]), float(row[1]), float(row[2])]
		#print(sim_data[-1][2] % 1)
		#check =  round(sim_data[-1][2] - sim_data[-2][2], 3)
		#print("current time:", float(row[2]), "sim_data[-1][2]: ", sim_data[-1][2], check)
		#if check == 0.002:
		odomhandler.simulate(sim_data)
			#print("plotted point")
		#print(row)

#odomhandler.simulate(sim_vel)
with open('position_square_20hz.csv') as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	for row in csv_reader:
		plt.plot(float(row[0]), float(row[1]), 'r+')

plt.show()


#print(odomhandler.arduino_data_processor.x_y_theta_t)
