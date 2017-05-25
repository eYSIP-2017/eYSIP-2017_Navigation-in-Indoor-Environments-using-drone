#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from visualization_msgs.msg import Marker
import numpy as np

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

take_off_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)

twist = Twist()
flag = True
def callback(data):
    global flag
    if flag:
        take_off_pub.publish()
        print('published take off')
        for i in range(200):
            twist.linear.z = 2
            pub.publish(twist)
        flag = False
    lastError = np.array([0.,0.])
    kp = 0.1
    kd = 5
    ki = 0
    integral = np.array([0.,0.])
    derivative = np.array([0.,0.])
 
    while 1:
        error = np.array([data.pose.position.x, data.pose.position.y])
        integral += error
        derivative = error - lastError;

        f = kp * error + ki * integral + kd * derivative;  
        
        lastError = error;
        
        twist.linear.x = f[0]
        twist.linear.y = f[1]
        
        pub.publish(twist)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hover_aruco_marker', anonymous=True)
    take_off_pub.publish()
    print('published take off')
    rospy.Subscriber("/Estimated_marker", Marker, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# runs the listener function if the file is run as a script
if __name__ == '__main__':
    listener()
