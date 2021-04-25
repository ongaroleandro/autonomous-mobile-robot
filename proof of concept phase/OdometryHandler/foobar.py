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
		self.arduino_data_processor = DataProcessor("arduino_data", 0.025, 0.210)
		self.pub = Publisher('odom', "odom", "base_link")

	def simulate(self,sim_data):
		for data in sim_data:
			wl, wr, t_ard = self.arduino_data_processor.extractData(data)

			self.arduino_data_processor.calcPoseTwist(wl, wr, t_ard)

			x_y_theta_t, vx_vth = self.arduino_data_processor.getPublisherInfo()

			msg = self.pub.createNavMsg(1200, x_y_theta_t, vx_vth)

			plt.plot(msg.pose.pose.position.x, msg.pose.pose.position.y, 'b.')
			#print(self.pub.createNavMsg(1200, x_y_theta_t, vx_vth))

	def main(self):
		current_time = self.pub.getCurrentTime()
		
		x_y_theta_t, vx_vth = self.arduino_data_processor.getPublisherInfo()
		self.pub.publishMessage(self.pub.createNavMsg(current_time, x_y_theta_t, vx_vth), self.pub.createTF(current_time, x_y_theta_t))


def startNode():

	rospy.init_node('odometry_handler', anonymous = False) #initialise node
	odomhandler = OdometryHandler() #create odometryhandler object
        #start while loop
        while not rospy.is_shutdown():
            rate = rospy.Rate(1) #adjust publishing rate here
            odomhandler.main()
            rate.sleep()
        rospy.spin() #stop node


if __name__ == "__main__":
	startNode()

'''
r_wheel = 0.025
wheel_sep = 0.210

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


def giveData():
	wl = 5
	wr = 10

	simulated_data = []
	delta_t = 0.1
	i = 0
	while i < 10:
		simulated_data.append([wl, wr, i])
		i += delta_t

	return simulated_data


fig = plt.figure()
odomhandler = OdometryHandler()
odomhandler.simulate(giveData())
plt.show()


#print(odomhandler.arduino_data_processor.x_y_theta_t)
'''
