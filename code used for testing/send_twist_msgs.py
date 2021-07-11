#!/usr/bin/env python

'''IMPORTANT:
This file needs to be used together with odometry_odometry_handler_collect_data.py if you want to be able to reset the odom topic.
You can find it in the repo in /code used for testing/OdometryHandler_collect_data/
'''

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
        self.rate = rospy.Rate(20.0)
        self.resetAfter = False
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.desired_distance = 0.0
        self.desired_angle = 0.0
        self.data = []

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


    def publish_vel(self, twist_msg):
        self.pub_vel.publish(twist_msg)

    def publish_reset(self):
        while not rospy.wait_for_message('odom', Odometry, timeout=1.0).pose.pose.position.x == 0.0:
            self.pub_reset.publish("reset")
            self.rate.sleep()

    def write_to_csv(self):
        np.savetxt('data.csv', self.data, fmt="%1.3f", delimiter=",", header='x,y', comments='') #the comments argument is needed because by default the header string will be preced by a # since the header, for numpy, is a comment

    def createTwistMsg(self, x, z):
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.angular.z = z

        return twist_msg

    def translate(self):
        available = False
        while not available and not rospy.is_shutdown():
            if self.tfBuffer.can_transform('odom', 'base_link', rospy.Time()):
                available = True

        start_transform = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time())
        start_xpos = start_transform.transform.translation.x
        start_ypos = start_transform.transform.translation.y

        done = False
        while not done and not rospy.is_shutdown():
            self.publish_vel(self.createTwistMsg(self.linear_x, 0.0))
            self.rate.sleep()
            try:
                current_transform = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

            current_xpos = current_transform.transform.translation.x
            current_ypos = current_transform.transform.translation.y

            if abs(current_xpos - start_xpos) >= self.desired_distance or abs(current_ypos - start_ypos) >= self.desired_distance:
                self.publish_vel(self.createTwistMsg(0.0, 0.0))
                if self.resetAfter:
                    msg = rospy.wait_for_message('odom', Odometry, timeout=1.0) # In theory we could also just use the current_transform
                    self.published_position.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
                    self.publish_reset()
                done = True

    def rotate(self):
        #checking if rotating clockwise or counter clockwise 
        if self.angular_z > 0:
            clockwise = False
        else:
            clockwise = True

        #checking if transform is available
        available = False
        while not available and not rospy.is_shutdown():
            if self.tfBuffer.can_transform('odom', 'base_link', rospy.Time()):
                available = True

        #getting start transform
        start_transform = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time())
        
        #getting the angle from the transform
        start_angle = tf_conversions.transformations.euler_from_quaternion([start_transform.transform.rotation.x, start_transform.transform.rotation.y, start_transform.transform.rotation.z, start_transform.transform.rotation.w])[2]

        #changing interval from [-pi, pi] to [0, 2*pi]
        if not clockwise and start_angle < 0:
            start_angle = start_angle + 2*np.pi
        elif clockwise and start_angle > 0:
            start_angle = 2*np.pi - start_angle
        
        done = False
        while not done and not rospy.is_shutdown():
            #publishing twist message with specified angular velocity
            self.publish_vel(self.createTwistMsg(0.0, self.angular_z))
            self.rate.sleep()
            
            #getting current transform
            try:
                current_transform = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

            #getting the angle from the transform
            current_angle = tf_conversions.transformations.euler_from_quaternion([current_transform.transform.rotation.x, current_transform.transform.rotation.y, current_transform.transform.rotation.z, current_transform.transform.rotation.w])[2]

            #changing interval from [-pi, pi] to [0, 2*pi]
            if not clockwise and current_angle < 0:
                current_angle = current_angle + 2*np.pi
            elif clockwise and current_angle > 0:
                current_angle = current_angle - 2*np.pi

            #calculating the angle between the transforms and converting it to degrees
            relative_angle = abs(current_angle - start_angle) * 180 / np.pi
            print('start_angle: {} current angle: {} angle_turned: {}'.format(start_angle*180/np.pi, current_angle*180/np.pi, relative_angle))

            if relative_angle >= self.desired_angle:
                self.publish_vel(self.createTwistMsg(0.0, 0.0))
                done = True

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

if __name__ == "__main__":
    rospy.init_node('send_twist_msgs_node', anonymous = False)

    comc = CommandCenter()

    print(bcolors.OKBLUE + 'Type "help" for all possible instructions.' + bcolors.ENDC)

    speeds, limits = False, False
    while not rospy.is_shutdown():
        #checking if speeds and desired distance/angle have been set
        if not speeds and not limits:
            print(bcolors.FAIL + 'PLEASE SET SPEEDS AND DISTANCE/ANGLE WITH "setSpeeds" and "setLimits"' + bcolors.ENDC)
        elif not speeds:
            print(bcolors.FAIL + 'PLEASE SET SPEEDS with "setSpeeds"' + bcolors.ENDC)
        elif not limits:
            print(bcolors.FAIL + 'PLEASE SET DESIRED DISTANCE AND ANGLE WITH "setLimits"' + bcolors.ENDC)

        #giving a warning if odom topic gets reset
        if comc.resetAfter:
            print(bcolors.WARNING + 'REMINDER: odom topic values will be reset to zero after a translation or rotation' + bcolors.ENDC)

        #getting user input
        user_input = raw_input('Give instruction: ')

        #checking what the user input is and acting accordingly
        if user_input == 'setSpeeds':
            speed_vel = raw_input('Give linear velocity (in m/s) and rotational velocity (in rad/s) separated by a comma: ')
            speed_vel = speed_vel.split(',')
            try:
                comc.linear_x = float(speed_vel[0])
                comc.angular_z = float(speed_vel[1])
                speeds = True
                print(bcolors.OKGREEN + 'Set linear_x to: {} m/s and angular_z to: {} rad/s'.format(speed_vel[0], speed_vel[1]) + bcolors.ENDC)
            except IndexError as e:
                print(bcolors.FAIL + 'IndexError: ' + str(e) + bcolors.ENDC)

        if user_input == 'setLimits':
            dist_ang = raw_input('Give desired distance (in m) and desired angle (in degrees) seperated by a comma: ')
            dist_ang = dist_ang.split(',')
            try:
                comc.desired_distance = float(dist_ang[0])
                comc.desired_angle = float(dist_ang[1])
                limits = True
                print(bcolors.OKGREEN + 'Set desired_distance to: {} m and desired_angle to: {} degrees'.format(dist_ang[0], dist_ang[1]) + bcolors.ENDC)
            except IndexError as e:
                print(bcolors.FAIL + 'IndexError: ' + str(e) + bcolors.ENDC)

        if user_input == 'toggleResetAfter':
            if comc.resetAfter:
                comc.resetAfter = False
            else:
                comc.resetAfter = True
            print(bcolors.OKGREEN + 'resetAfter has been set to: {}'.format(comc.resetAfter) + bcolors.ENDC)

        if user_input == 'publishReset':
            comc.publish_reset()
     
        if user_input == 'go':
            comc.translate()

        if user_input == 'rotate':
            comc.rotate()

        if user_input == 'write':
            comc.write_to_csv()
                                    
        if user_input == 'stop':
            rospy.signal_shutdown("typed stop")

        if user_input == 'help':
            instructions = [['Instruction', 'Description'],
                            ['-----------', '-----------'],
                            ['setSpeeds', 'Specify the linear and angular velocity.'],
                            ['setLimits', 'Specify the desired translation distance and rotation angle.'],
                            ['toggleResetAfter', 'Reset the odom topic to zero after translating.'],
                            ['publishReset', 'Manually reset the odom topic to zero.'],
                            ['go', 'Make the robot translate until desired distance is reached.'],
                            ['rotate', 'Make the robot rotate until the desired angle is reached.'],
                            ['write', 'Write data list to csv file (called data.csv).'],
                            ['stop', 'Stop the node.']]
            for i in range(len(instructions)):
                print('{:<20} {:<8}'.format(instructions[i][0], instructions[i][1]))


