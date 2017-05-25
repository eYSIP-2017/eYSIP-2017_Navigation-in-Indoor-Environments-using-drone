#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from ardrone_autonomy.msg import Navdata
from visualization_msgs.msg import Marker
import numpy as np

pubx = rospy.Publisher('plot_x', Float64, queue_size=5)
puby = rospy.Publisher('plot_y', Float64, queue_size=5)
pubz = rospy.Publisher('plot_z', Float64, queue_size=5)
pub_yaw = rospy.Publisher('plot_yaw', Float64, queue_size=5)

def pid(data, state):
    set_array = np.array([0.2, 0, 0, 1.57])
    error = np.array([data[0], data[1], data[2], data[3]]) - set_array

    pubx.publish(error[0])
    puby.publish(error[1])
    pubz.publish(error[2])
    pub_yaw.publish(error[3])

    print(error)
    state['integral'] += error
    state['derivative'] = error - state['lastError']

    f = state['p'] * error + state['i'] * state['integral'] + state['d'] * state['derivative']  
    
    state['lastError'] = error
    
    twist = Twist()
    # twist.linear.x = -f[0]
    # twist.linear.y = -f[1]
    # twist.linear.z = -f[2]
    twist.angular.z = -f[3]
    return twist, state
