#!/usr/bin/env python

#IMPORTANT: this file needs to be used together with odometry_odometry_handler_collect_data.py
#           You can find it in /code used for testing/OdometryHandler_collect_data/

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf_conversions
import tf2_ros

class CommandCenter(object):

    def __init__(self):
        self.pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.pub_reset = rospy.Publisher("command", String, queue_size = 1) #odometry_odometry_handler_collect_data.py subscribes to this topic
        self.sub = rospy.Subscriber("odom", Odometry, self.msgCallback)
        self.twist_msg = Twist()
        self.published_position = []

    def publish_vel(self):
        self.pub_vel.publish(self.twist_msg)

    def publish_reset(self):
        self.pub_reset.publish("reset")

    def write_to_csv(self):
        np.savetxt('test_data.csv', self.published_position, fmt="%1.3f", delimiter=",", header='x,y,theta', comments='') #the comments argument is needed because by default the header string will be preced by a # since the header, for numpy, is a comment

    def msgCallback(self, msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.x
        self.quat_z = msg.pose.pose.orientation.z 


if __name__ == "__main__":
    rospy.init_node('send_twist_msgs_node', anonymous = False)

    comc = CommandCenter()

    while not rospy.is_shutdown():

        user_input = raw_input('Give instruction: ')

        if user_input == 'reset':
            comc.publish_reset()

        if user_input == 'print':
            print(comc.published_position)

        if user_input == 'write':
            comc.write_to_csv()
                                    
        if user_input == 'stop':
            rospy.signal_shutdown("typed stop")

        if user_input == 'go':
            comc.twist_msg.linear.x = 0.2
            comc.twist_msg.angular.z = 0.0
            comc.publish_vel()

            rospy.sleep(5)

            comc.twist_msg.linear.x = 0
            comc.twist_msg.angular.z = 0.0
            comc.publish_vel()

            comc.twist_msg.linear.x = 0
            comc.twist_msg.angular.z = 1.0
            comc.publish_vel()

            rospy.sleep(4)

            comc.twist_msg.linear.x = 0
            comc.twist_msg.angular.z = 0.0
            comc.publish_vel()

            msg = rospy.wait_for_message('odom', Odometry, timeout=1.0)

            theta = tf_conversions.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

            comc.published_position.append([msg.pose.pose.position.x, msg.pose.pose.position.y, theta])
            
            rospy.sleep(0.2)

            comc.publish_reset()

        if user_input == 'rotate':
            comc.twist_msg.linear.x = 0
            comc.twist_msg.angular.z = 0.6
            comc.publish_vel()
            while not comc.quat_z < 0:
                print("current z: ", comc.quat_z)

            comc.twist_msg.linear.x = 0
            comc.twist_msg.angular.z = 0.0
            comc.publish_vel()


    rospy.spin()

  
