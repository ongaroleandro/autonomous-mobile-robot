#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt

x_y_theta_t = [[0, 0, 0, 0]]
r_wheel = 0.025
wheel_sep = 0.210


def calc_pos(msg):

    wl = msg[0]
    wr = msg[1]
    t_ard = msg[2]
    delta_theta = (-wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) / wheel_sep
    delta_s = (wl * r_wheel * (t_ard - x_y_theta_t[-1][3]) + wr * r_wheel * (t_ard - x_y_theta_t[-1][3])) * 0.5
    x = x_y_theta_t[-1][0] + delta_s * np.cos(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    y = x_y_theta_t[-1][1] + delta_s * np.sin(x_y_theta_t[-1][2]*np.pi/180 + delta_theta * 0.5)
    theta = x_y_theta_t[-1][2] + delta_theta*180/np.pi #FIX ME: theta needs to stay between -359 and +359 degrees

    x_y_theta_t.append([x, y, theta, t_ard])

#turning 45 degrees 2 times
#sim_vel = [[10, 10, 1],
#           [10, 10, 2],
#           [10, 10, 3],
#           [10, 10, 4],
#           [(wheel_sep*0.5*np.pi)/(2*r_wheel),  0, 5],
#           [10, 10, 6],
#           [10, 10, 7],
#           [10, 10, 8],
#           [10, 10, 9],
#           [(wheel_sep*0.5*np.pi)/(2*r_wheel), 0, 10],
#           [10, 10, 11],
#           [10, 10, 12],
#           [10, 10, 13],
#           [10, 10, 14]]

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

for i in range(len(sim_vel)):
    calc_pos(sim_vel[i])

print(*x_y_theta_t, sep='\n')

fig = plt.figure()
for i in range(len(x_y_theta_t)):
    plt.plot(x_y_theta_t[i][0], x_y_theta_t[i][1], 'b.')

plt.show()

#same code but using numpy array and not making the array bigger by appending every new position
'''
import numpy as np
from matplotlib import pyplot as plt

x_y_theta_t = np.array([0, 0, 0, 0])
r_wheel = 0.025
wheel_sep = 0.210

print(x_y_theta_t)
fig = plt.figure()

def calc_pos(msg):
    global x_y_theta_t
    wl = msg[0]
    wr = msg[1]
    t_ard = msg[2]
    delta_theta = (-wl * r_wheel * (t_ard - x_y_theta_t[3]) + wr * r_wheel * (t_ard - x_y_theta_t[3])) / wheel_sep
    delta_s = (wl * r_wheel * (t_ard - x_y_theta_t[3]) + wr * r_wheel * (t_ard - x_y_theta_t[3])) * 0.5
    x = x_y_theta_t[0] + delta_s * np.cos(x_y_theta_t[2]*np.pi/180 + delta_theta * 0.5)
    y = x_y_theta_t[1] + delta_s * np.sin(x_y_theta_t[2]*np.pi/180 + delta_theta * 0.5)
    theta = x_y_theta_t[2] + delta_theta*180/np.pi #FIX ME: theta needs to stay between -359 and +359 degrees
    
    x_y_theta_t = np.array([x, y, theta, t_ard])
    print(x_y_theta_t)
    plt.plot(x, y, 'b.')

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

for i in range(len(sim_vel)):
    calc_pos(sim_vel[i])
    
plt.show()
'''