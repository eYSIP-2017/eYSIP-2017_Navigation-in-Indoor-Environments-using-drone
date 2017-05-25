#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from ardrone_autonomy.msg import Navdata

import sys, select, termios, tty
from pid import pid
import numpy as np

msg = """
Control Your AR Drone!
---------------------------
Moving around:   
        i    
   j    k    l
   
   y - up
   h - down

   u - counterclockwise
   o - clockwise

t : takeoff
g : land

CTRL-C to quit
"""

moveBindings = {
        # forward and back
        'i':(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        'k':(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        # left and right
        'j':(0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
        'l':(0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
        # up and down
        'y':(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        'h':(0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
        # counterclockwise and clockwise
        'u':(0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        'o':(0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
        # stop
        'm':((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
           }


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

coords = np.array([0.0,0.0,0.0,0.0])

def get_pose_from_aruco(data):
    global coords, yaw
    quaternion = (data.pose.orientation.x,
                  data.pose.orientation.y,
                  data.pose.orientation.z,
                  data.pose.orientation.w
                  )
    euler = euler_from_quaternion(quaternion)
    
    coords[2] = data.pose.position.x
    coords[1] = data.pose.position.y
    coords[0] = data.pose.position.z
    # coords[3] = euler[2]

def get_angle_from_navdata(data):
    global coords
    coords[3] = data.rotZ if data.rotZ > 0 else 360 + data.rotZ

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('ardrone_teleop')
    rospy.Subscriber("/Estimated_marker", Marker, get_pose_from_aruco)
    rospy.Subscriber("/ardrone/navdata", Navdata, get_angle_from_navdata)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    take_off_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)

    xyz = (0,0,0,0,0,0)
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print msg
        print vels(speed,turn)
        # state dict for pid
        state = dict()
        state['lastError'] = np.array([0.,0.,0.,0.])

        # values of x and y may remain same
        state['p'] = np.array([1, 1, 1, 0.1], dtype=float)
        state['i'] = np.array([0, 0, 0, 0], dtype=float)
        state['d'] = np.array([0, 0, 0, 0], dtype=float)

        state['integral'] = np.array([0.,0.,0.,0.])
        state['derivative'] = np.array([0.,0.,0.,0.])

        twist = Twist()
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                xyz = moveBindings[key]
                count = 0
            elif key == 't':
                take_off_pub.publish()
            elif key == 'g':
                land_pub.publish()
            elif key == 'p':
                while 1:
                    pid_twist, state = pid(coords, state)
                    pub.publish(pid_twist)
                    key = getKey()
                    if key == 's':
                        state['lastError'] = np.array([0.,0.,0.,0.])
                        state['integral'] = np.array([0.,0.,0.,0.])
                        state['derivative'] = np.array([0.,0.,0.,0.])
                        break
            # set yaw pid consts
            elif key == 'e':
                pid_consts = input()
                state['p'][3] = float(pid_consts[0])
                state['i'][3] = float(pid_consts[1])
                state['d'][3] = float(pid_consts[2])
            # set z axis pid consts
            elif key == 'd':
                pid_consts = input()
                state['p'][2] = float(pid_consts[0])
                state['i'][2] = float(pid_consts[1])
                state['d'][2] = float(pid_consts[2])
            # set xy axis pid consts
            elif key == 'c':
                pid_consts = input()
                state['p'][0] = float(pid_consts[0])
                state['i'][0] = float(pid_consts[1])
                state['d'][0] = float(pid_consts[2])
                state['p'][1] = float(pid_consts[0])
                state['i'][1] = float(pid_consts[1])
                state['d'][1] = float(pid_consts[2])
            elif key == ' ':
                xyz = (0,0,0,0,0,0)
            elif key == 'f':
                print(state['p'][2], state['i'][2], state['d'][2])
            else:
                # count = count + 1
                # if count > 4:
                #     xyz = (0,0,0,0,0,0)
                if (key == '\x03'):
                    break


            twist.linear.x = xyz[0]
            twist.linear.y = xyz[1]
            twist.linear.z = xyz[2]
            twist.angular.x = xyz[3]
            twist.angular.y = xyz[4]
            twist.angular.z = xyz[5]
            pub.publish(twist)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    # except Exception as e:
    #     print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        land_pub.publish()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

