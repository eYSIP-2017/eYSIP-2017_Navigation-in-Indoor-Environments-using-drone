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
from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist, Pose, Vector3Stamped
from std_msgs.msg import Empty, Float64
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from ardrone_autonomy.msg import Navdata
from aruco_mapping.msg import *
from pose import pose

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


marker_pose = pose()
global_pose = pose()

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


def get_angle_from_navdata(data):
    global coords
    # # mag = np.array([data.vector.x, data.vector.y])
    # mag = np.array([data.magX, data.magY])
    # # target = np.array([0.50, 0.])
    # target = np.array([0., 30.])
    # target[0] = target[0] if target[0] >= 0 else 0

    # coords[3] = np.arccos(np.sum(mag*target)/(np.sqrt(np.sum(np.square(mag)))* np.sqrt(np.sum(np.square(target)))))

    coords[3] = data.rotZ if data.rotZ > 0 else 360 + data.rotZ
    # temp_pub.publish(coords[3])

def check_battery(data):
    if data.batteryPercent < 15:
        land_pub.publish()

coords = np.array([0.0,0.0,0.0,0.0])

max_found = False

def get_aruco_pose(temp_pose):
    global max_found
    marker_pose.store_marker_ids(temp_pose.marker_ids)
    if len(temp_pose.marker_ids) != 0:
        if max_found == True:
            current_marker_id = min(temp_pose.marker_ids)
        else:
            current_marker_id = max(temp_pose.marker_ids)

        if current_marker_id == 16:
            max_found = True
    # if marker_pose.get_current_marker_id() is not None and len(temp_pose.marker_ids) != 0:
        marker_pose.convert_geometry_transform_to_pose(temp_pose.global_marker_poses[temp_pose.marker_ids.index(current_marker_id)])

    global_pose.convert_geometry_transform_to_pose(temp_pose.global_camera_pose)


# set_array = None
# def get_aruco_pose(temp_pose):
#     global tran_x
#     quaternion = (temp_pose.global_camera_pose.orientation.x,
#                   temp_pose.global_camera_pose.orientation.y,
#                   temp_pose.global_camera_pose.orientation.z,
#                   temp_pose.global_camera_pose.orientation.w
#                   )
#     euler = euler_from_quaternion(quaternion)
#     coords[0] = temp_pose.global_camera_pose.position.x
#     coords[1] = temp_pose.global_camera_pose.position.y
#     coords[2] = temp_pose.global_camera_pose.position.z
    # coords[3] = euler[0] #temp_pose.global_camera_pose.orientation.z

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    marker_pose = pose()
    global_pose = pose()
    # print(last_time, dt)
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('ardrone_teleop')
    aruco_front = bool(rospy.get_param('~aruco_front', 'true'))
    rospy.Subscriber("/ardrone/navdata", Navdata, check_battery)

    # rospy.Subscriber("/Estimated_marker", Marker, get_pose_from_aruco)

    rospy.Subscriber('aruco_poses', ArucoMarker, get_aruco_pose)
    # rospy.Subscriber("/ardrone/navdata", Navdata, get_angle_from_navdata)
    # rospy.Subscriber("/magnetic", Vector3Stamped, get_angle_from_navdata)
    
    temp_pub = rospy.Publisher('/yaw', Float64, queue_size=5)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    take_off_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)
    reset_pub = rospy.Publisher('/ardrone/reset', Empty, queue_size=5)

    marker_pose = pose()
    global_pose = pose()
    marker_ids = marker_pose.get_marker_ids()

    ori_z = 0
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
        print(msg)
        print(vels(speed,turn))
        # state dict for pid
        state = dict()
        state['lastError'] = np.array([0.,0.,0.,0.])

        # values of x and y may remain same
        xy_pid = [0.2, 0.01, 0.05]
        xy_pid_bottom = [2, 0., 0.2]
        # xy_pid = [1, 0.0, 0.0]
        if aruco_front:
            state['p'] = np.array([xy_pid[0], xy_pid[0], 1, 0.6], dtype=float)
            state['i'] = np.array([xy_pid[1], xy_pid[1], 0.1, 0.1], dtype=float)
            state['d'] = np.array([xy_pid[2], xy_pid[2], 1, 0.05], dtype=float)
            # state['p'] = np.array([xy_pid[0], xy_pid[0], 1, 0.6], dtype=float)
            # state['i'] = np.array([xy_pid[1], xy_pid[1], 0., 0.], dtype=float)
            # state['d'] = np.array([xy_pid[2], xy_pid[2], 0, 0.], dtype=float)
        else:
            state['p'] = np.array([xy_pid_bottom[0], xy_pid_bottom[0], 1, 0.5], dtype=float)
            state['i'] = np.array([xy_pid_bottom[1], xy_pid_bottom[1], 0.1, 0.1], dtype=float)
            state['d'] = np.array([xy_pid_bottom[2], xy_pid_bottom[2], 1, 0.05], dtype=float)

        state['integral'] = np.array([0.,0.,0.,0.])
        state['derivative'] = np.array([0.,0.,0.,0.])
        yaw_set = 180

        twist = Twist()
        import time
        while(1):
            state['last_time'] = time.time()
            key = getKey()
            if key in moveBindings.keys():
                xyz = moveBindings[key]
                count = 0
            elif key == 't':
                take_off_pub.publish()
            elif key == 'g':
                land_pub.publish()
            elif key == 'w':
                import follow_trajectory as ft
                import actionlib
                import drone_application.msg
                client = actionlib.SimpleActionClient('move_to_waypoint', drone_application.msg.moveAction)
                waypoints = [[0,0,1,0], [1,0,1,0], [2,0,1,0]]
                for waypoint in waypoints:
                    print(waypoint)
                    result = ft.send_goal(waypoint, client)
                    print(result)
            elif key == 'q':
                # max_found = False
                # min_found = False
                # marker_ids = marker_pose.get_marker_ids()
                last_twist = Twist()
                feed_stuck_count = 0
                twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                while(1):
                    set_array = marker_pose.as_waypoints()
                    set_array[0] += 1.5
                    # print(max_found, set_array, marker_pose.get_current_marker_id())
                    current_pose = global_pose.as_waypoints()
                    if (current_pose == np.array([0., 0., 0., 0.])).all():
                        pub.publish(twist)
                    else:
                        pid_twist, state = pid(current_pose, state, aruco_front, yaw_set, set_array)
                        
                        if last_twist == pid_twist:
                            feed_stuck_count += 1

                        if feed_stuck_count > 2:
                            pub.publish(twist)
                            feed_stuck_count = 0
                        else:
                            pub.publish(pid_twist)

                        last_twist = pid_twist
                    key = getKey()
                    if key == 'a':
                        state['lastError'] = np.array([0.,0.,0.,0.])
                        state['integral'] = np.array([0.,0.,0.,0.])
                        state['derivative'] = np.array([0.,0.,0.,0.])
                        xyz = (0,0,0,0,0,0)
                        break
                    elif key == 'f':
                        print('yaw: {}, {}, {}; z-axis: {}, {}, {}; xy-axis: {}, {}, {};'.format(state['p'][3], state['i'][3], state['d'][3],
                             state['p'][2], state['i'][2], state['d'][2], state['p'][0], state['i'][0], state['d'][0]))
                        print('e - yaw;     d - z_axis;     c - xy_axis')
                    elif key == 'g':
                        land_pub.publish()
                        xyz = (0,0,0,0,0,0)
                        break

            elif key == 'p':
                set_array = np.array([1, tran_y, 0, 0])
                # set_array = [0.5, 0.1 ,-0.1, 0]
                # set_array = [coords[0], coords[1], coords[2], coords[3]]
                while 1:
                    set_array = np.array([1, tran_y, 0, 0])
                    pid_twist, state = pid(coords, state, aruco_front, yaw_set, set_array)
                    pub.publish(pid_twist)

                    # set_array = [tran_x, tran_y, tran_z, ori_z]
                    # pid_twist, state = pid(coords, state, aruco_front, yaw_set) #, set_array)
                    # pub.publish(pid_twist)
                    key = getKey()
                    if key == 's':
                        state['lastError'] = np.array([0.,0.,0.,0.])
                        state['integral'] = np.array([0.,0.,0.,0.])
                        state['derivative'] = np.array([0.,0.,0.,0.])
                        xyz = (0,0,0,0,0,0)
                        break
                    elif key == 'f':
                        print('yaw: {}, {}, {}; z-axis: {}, {}, {}; xy-axis: {}, {}, {};'.format(state['p'][3], state['i'][3], state['d'][3],
                             state['p'][2], state['i'][2], state['d'][2], state['p'][0], state['i'][0], state['d'][0]))
                        print('e - yaw;     d - z_axis;     c - xy_axis')
                    elif key == 'g':
                        land_pub.publish()
                        xyz = (0,0,0,0,0,0)
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
            elif key == 'x':
            	state['i'][0] = input()
            	state['i'][1] = state['i'][0]
            elif key == ' ':
                reset_pub.publish()
            elif key == 'f':
                # print(state['p'][3], state['i'][3], state['d'][3], )
                print('yaw: {}, {}, {}; z-axis: {}, {}, {}; xy-axis: {}, {}, {};'.format(state['p'][3], state['i'][3], state['d'][3],
                     state['p'][2], state['i'][2], state['d'][2], state['p'][0], state['i'][0], state['d'][0]))
                print('e - yaw;     d - z_axis;     c - xy_axis')
            else:
                # count = count + 1
                # if count > 4:
                # xyz = (0,0,0,0,0,0)
                if (key == '\x03'):
                    break

            # xyz = [xyz[i] * 0.5 for i in range(6)]
            xyz = np.clip(np.array(xyz), -0.3, 0.3)
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

