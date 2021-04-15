#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros


x_y_theta_t = [[0, 0, 0, 0]]
r_wheel = 0.025
wheel_sep = 0.210


def calc_pos(msg):
    if (len(x_y_theta_t) == 1):
        x_y_theta_t.append([0, 0, 0, msg.data[2]])
    wl = msg.data[0]*np.pi/180
    wr = msg.data[1]*np.pi/180
    t_ard = msg.data[2]
   
    delta_theta = (-wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) / wheel_sep
    delta_s = (wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) * 0.5
    x = x_y_theta_t[-1][0] + delta_s * np.cos(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    y = x_y_theta_t[-1][1] + delta_s * np.sin(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    theta = x_y_theta_t[-1][2] + delta_theta*180/np.pi #FIX ME: theta needs to stay between -359 and +359 degrees
    
    x_y_theta_t.append([x, y, theta, t_ard])

    current_time = rospy.Time.now()

    #creating the tf-broadcaster and populating it with data
    odom_broadcaster = tf2_ros.TransformBroadcaster()
    odom_trans = geometry_msgs.msg.TransformStamped()

    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"
    odom_trans.transform.translation.x = x
    odom_trans.transform.translation.y = y
    odom_trans.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta)
    odom_trans.transform.rotation.x = q[0]
    odom_trans.transform.rotation.x = q[1]
    odom_trans.transform.rotation.y = q[2]
    odom_trans.transform.rotation.w = q[3]

    odom_broadcaster.sendTransform(odom_trans)

    #publishing the odometry message
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0

    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = 0 #todo: calculate vx, vy and vth from wl and wr
    odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.z = 0
    
    odom_pub.publish(odom)
    rate = rospy.Rate(1)
    rate.sleep()


def write_to_csv():
    np.savetxt('data.csv', x_y_theta_t, fmt="%1.3f", delimiter=',')


def init_node():
    rospy.init_node('odometry_handler', anonymous=True)  # anonymous=True appends a unique ID to the node name
    rospy.Subscriber("arduino_data", Float32MultiArray, calc_pos)
    rospy.on_shutdown(write_to_csv)
    rospy.spin()


if __name__ == '__main__':
    init_node()
