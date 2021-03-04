#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt

pos_time = []
def plot(msg):

    #speed_ang = msg.angular.z
    speed_lin = msg.linear.x
    t_zero = int(rospy.Time.now().to_sec())
    pos_time.append([speed_lin, t_zero])
    rospy.loginfo(pos_time)
'''
    while speed_lin != 0:
        t1 = rospy.Time.now().to_sec()
        distance = speed_lin*(t1-t0)
        x_pos.append(distance)
        time.append(t1-t0)
	rospy.loginfo(distance)
'''
    #print('x_pos:',x_pos)
    #print('time:',time)

def init_node():
    rospy.init_node('plot_cmd_vel', anonymous=True) #anonymous=True appends a unique ID to the node name, i.e. plot_cmd_vel 
    rospy.Subscriber("cmd_vel", Twist, plot)
    rospy.spin()

if __name__ == '__main__':
    init_node()
