#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist

file = open('cmd_vel_log.txt', 'w')
file.truncate()

def store_to_file(data):
	file.write(str(data.linear.x))
	file.write('\t')
	file.write(str(data.linear.y))
	file.write('\t')
	file.write(str(data.linear.z))
	file.write('\n')

try:
	rospy.init_node('logging')
	pub = rospy.Subscriber('/cmd_vel', Twist, store_to_file)

	rospy.spin()

finally:
	file.close()