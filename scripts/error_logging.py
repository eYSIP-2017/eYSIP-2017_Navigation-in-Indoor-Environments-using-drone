#!/usr/bin/env python
from __future__ import print_function

import rospy
from drone_application.msg import pid_error
import numpy as np

file = open('error_log.txt', 'w')
file.truncate()

def store_to_file(data):
	for dat in data.error:
		file.write(str(dat))
		file.write('      ')
	file.write('\n')

try:
	rospy.init_node('logging_pid')
	pub = rospy.Subscriber('/pid_error', pid_error, store_to_file)

	rospy.spin()

finally:
	file.close()