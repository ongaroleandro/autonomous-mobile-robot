#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros

from dataprocessor import DataProcessor
from publisher import Publisher

class OdometryHandler(object):

	def __init__(self):
		self.arduino_data_processor = DataProcessor("arduino_data", 0.025, 0.210)
		self.pub = Publisher('odom', "odom", "base_link")

	def main(self):
		current_time = self.pub.getCurrentTime()
		
		x_y_theta_t, vx_vth = self.arduino_data_processor.getPublisherInfo()
		if self.arduino_data_processor.arduino_data is not None:
			self.pub.publishMessage(self.pub.createNavMsg(current_time, x_y_theta_t, vx_vth), self.pub.createTF(current_time, x_y_theta_t))


def startNode():

	rospy.init_node('odometry_handler', anonymous = False) #initialise node
	odomhandler = OdometryHandler() #create odometryhandler object
    rate = rospy.Rate(20) #adjust publishing rate here
    #start while loop
    while not rospy.is_shutdown():
        odomhandler.main()
        rate.sleep()
    rospy.spin() #stop node


if __name__ == "__main__":
	startNode()
