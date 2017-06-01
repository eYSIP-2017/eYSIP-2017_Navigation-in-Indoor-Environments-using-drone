#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Empty, Float64
from ardrone_autonomy.msg import Navdata
from visualization_msgs.msg import Marker
import numpy as np

def callback(data):
    mag = np.array([data.magX, data.magY])
    target = np.array([0.58, 0.07])
    # target[0] = target[0] if target[0] >= 0 else 0
    print(np.sum(mag*target))
    final = np.arccos(np.sum(mag*target)/(np.sqrt(np.sum(np.square(mag)))* np.sqrt(np.sum(np.square(target)))))
    # print(final)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hover_aruco_marker', anonymous=True)
    # rospy.Subscriber("/magnetic", Vector3Stamped, callback)
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# runs the listener function if the file is run as a script
if __name__ == '__main__':
    listener()
