#!/usr/bin/env python
from __future__ import print_function

import rospy
from drone_application.msg import pid_error

file = open('five_markers.txt', 'w')
file.truncate()

def store_to_file(data):
    data = str(data)
    file.write(data[8: len(data) - 3])
    file.write('\n')

try:
    rospy.init_node('logging')
    rospy.Subscriber('/whycon_pose', pid_error, store_to_file)
    rospy.spin()

finally:
    file.close()