#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros

class Publisher(object):

	def __init__(self, frame_id, child_frame_id):

		self.frame_id = frame_id
		self.child_frame_id = child_frame_id

	def getCurrentTime(self):
		return rospy.Time.now()

	def createTF(self, current_time, x_y_theta_t):
		odom_trans = geometry_msgs.msg.TransformStamped()

		odom_trans.header.stamp = current_time
		odom_trans.header.frame_id = self.frame_id
		odom_trans.child_frame_id = self.child_frame_id				
		odom_trans.transform.translation.x = x_y_theta_t[0]
		odom_trans.transform.translation.y = x_y_theta_t[1]
		odom_trans.transform.translation.z = 0.0
		
		q = tf_conversions.transformations.quaternion_from_euler(0, 0, x_y_theta_t[2]*np.pi/180)
		
		odom_trans.transform.rotation.x = q[0]
		odom_trans.transform.rotation.y = q[1]
		odom_trans.transform.rotation.z = q[2]
		odom_trans.transform.rotation.w = q[3]

		return odom_trans

	def createNavMsg(self, current_time, x_y_theta_t, vx_vth):
		odom = Odometry()

		odom.header.stamp = current_time	
		odom.header.frame_id = self.frame_id
		odom.pose.pose.position.x = x_y_theta_t[0]
		odom.pose.pose.position.y = x_y_theta_t[1]
		odom.pose.pose.position.z = 0.0

		q = tf_conversions.transformations.quaternion_from_euler(0, 0, x_y_theta_t[2]*np.pi/180)

		odom.pose.pose.orientation.x = q[0]
		odom.pose.pose.orientation.y = q[1]
		odom.pose.pose.orientation.z = q[2]
		odom.pose.pose.orientation.w = q[3]

		odom.child_frame_id = self.child_frame_id
		odom.twist.twist.linear.x = vx_vth[0]
		odom.twist.twist.linear.y = 0	
		odom.twist.twist.linear.z = 0
		odom.twist.twist.angular.x = 0
		odom.twist.twist.angular.y = 0
		odom.twist.twist.angular.z = vx_vth[1]

		return odom

#pub = Publisher("odom", "base_link")
#print(pub.createNavMsg(10, [1,1,0,10], [1,5]))


