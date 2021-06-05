#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class move(object):

    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.pub2 = rospy.Publisher("command", String, queue_size = 1)
        self.move = Twist()
        self.published_position = [[0, 0]]

    def publish_vel(self):
        self.pub.publish(self.move)

    def publish_cmd(self):
        self.pub2.publish("reset")

    def write_to_csv(self):
        np.savetxt('test_data.csv', self.published_position, fmt="%1.3f", delimiter=",")


if __name__ == "__main__":
    rospy.init_node('move_node', anonymous = False)

    mov = move()

    while not rospy.is_shutdown():
        user_input = raw_input('Type: ')

        if user_input == 'reset':
            mov.publish_cmd()

        if user_input == 'print':
            print(mov.published_position)

        if user_input == 'write':
            mov.write_to_csv()
                
        if user_input == 'stop':
            rospy.signal_shutdown("typed stop")

        if user_input == 'start':
            mov.move.linear.x = 0.1
            mov.move.angular.z = 0.0

            mov.publish_vel()

            rospy.sleep(10.)

            mov.move.linear.x = 0
            mov.move.angular.z = 0.0

            mov.publish_vel()

            msg = rospy.wait_for_message('odom', Odometry, timeout=1.0)

            mov.published_position.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    rospy.spin()
