#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from drone_application.msg import pid_error
from ardrone_autonomy.msg import Navdata
from visualization_msgs.msg import Marker
import numpy as np

pubx = rospy.Publisher('plot_x', Float64, queue_size=5)
puby = rospy.Publisher('plot_y', Float64, queue_size=5)
pubz = rospy.Publisher('plot_z', Float64, queue_size=5)
pub_yaw = rospy.Publisher('plot_yaw', Float64, queue_size=5)

error_pub = rospy.Publisher('pid_error', pid_error, queue_size=5)

def pid(data, state, aruco_front):
    if aruco_front:
        set_array = np.array([1., 0., 0., 0.])
    else:
        set_array = np.array([0., 0., 1., 0.])
    error = np.array([data[0], data[1], data[2], data[3]]) - set_array

    pubx.publish(error[0])
    puby.publish(error[1])
    pubz.publish(error[2])
    pub_yaw.publish(error[3])

    error_pub.publish(error)
    state['integral'] += error
    state['derivative'] = error - state['lastError']

    f = state['p'] * error + state['i'] * state['integral'] + state['d'] * state['derivative']
    
    state['lastError'] = error
    
    # max_yaw = 1.
    # f[3] = f[3] if f[3] < 1 else max_yaw

    twist = Twist()
    # if error[3] > 1.5:
    #     twist.angular.z = -f[3]
    # elif error[2] > 0.1:
    #     # twist.linear.z = -f[2]
    #     twist.angular.z = -f[3]
    # else:
    if aruco_front:
        twist.linear.x = f[0] * np.cos(data[3])
        twist.linear.y = -f[1] * np.cos(data[3])
        twist.linear.z = -f[2]
        twist.angular.z = f[3]
    else:
        if error[0] > 0.1 or error[0] < -0.1 or error[1] > 0.1 or error[1] < -0.1 or error[2] > 0.1 or error[2] < -0.1:
            twist.linear.x = -f[0] * np.cos(data[3])
            twist.linear.y = -f[1] * np.cos(data[3])
            twist.linear.z = -f[2]
        else:
            twist.linear.x = -f[0] * np.cos(data[3])
            twist.linear.y = -f[1] * np.cos(data[3])
            twist.linear.z = -f[2]
            twist.angular.z = -f[3]
    return twist, state
