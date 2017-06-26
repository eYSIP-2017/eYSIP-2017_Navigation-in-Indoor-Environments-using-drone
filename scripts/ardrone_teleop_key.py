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

from geometry_msgs import msg
from std_msgs.msg import Empty, Float64
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from ardrone_autonomy.msg import Navdata
# from aruco_mapping.msg import *
from pose import Pose

import sys, select, termios, tty
from pid import pid
import numpy as np



doc_msg = """
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

def check_battery(data):
    if data.batteryPercent < 15:
        land_pub.publish()

def get_pose_from_aruco(temp_pose):
    if aruco_mapping:
        marker_pose.store_marker_ids(temp_pose.marker_ids)
        if len(temp_pose.marker_ids) != 0:
            if marker_pose.get_max_found():
                current_marker_id = min(temp_pose.marker_ids)
                if current_marker_id == 12:
                    pub.publish(twist)
                    land_pub.publish()
            else:
                current_marker_id = max(temp_pose.marker_ids)

            if current_marker_id == 17:
                marker_pose.set_max_found(True)
        # if marker_pose.get_current_marker_id() is not None and len(temp_pose.marker_ids) != 0:
            marker_pose.convert_geometry_transform_to_pose(temp_pose.global_marker_poses[temp_pose.marker_ids.index(current_marker_id)], aruco_mapping, aruco_front)

        global_pose.convert_geometry_transform_to_pose(temp_pose.global_camera_pose, aruco_mapping, aruco_front)
    else:
        global_pose.convert_geometry_transform_to_pose(temp_pose.pose, aruco_mapping, aruco_front)

def get_pose_from_kalman(kalman_pose):
    global_pose.convert_geometry_transform_to_pose(kalman_pose)

def get_pose_from_whycon(whycon_pose):
    whycon_pose = whycon_pose.poses[0]
    # x - +ve left of camera
    # y - +ve below of camera
    # z - higher away from camera
    # yaw - euler[1] - 1.57 at centre decreasing towards edges
    global_pose.convert_geometry_transform_to_pose(whycon_pose, remap=['z', 'x', 'y', 1])
    global_pose.x *= -1
    pose_pub.publish(global_pose.as_waypoints())

if __name__=="__main__":
    marker_pose = Pose()
    global_pose = Pose()
    # print(last_time, dt)
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('ardrone_teleop')
    aruco_front = bool(rospy.get_param('~aruco_front', 'true'))
    aruco_mapping = bool(rospy.get_param('~aruco_mapping', 'true'))
    localisation = bool(rospy.get_param('~localisation', 'true'))

    rospy.Subscriber("/ardrone/navdata", Navdata, check_battery)
    rospy.Subscriber('/whycon/poses', msg.PoseArray, get_pose_from_whycon)
    if localisation:
        rospy.Subscriber('kalman_pose', msg.Pose, get_pose_from_kalman)
    # if aruco_mapping:
    #     rospy.Subscriber('aruco_poses', ArucoMarker, get_pose_from_aruco)
    # else:
    #     rospy.Subscriber("/Estimated_marker", Marker, get_pose_from_aruco)

    # rospy.Subscriber("/ardrone/navdata", Navdata, get_angle_from_navdata)
    from drone_application.msg import pid_error

    temp_pub = rospy.Publisher('/yaw', Float64, queue_size=5)
    pose_pub = rospy.Publisher('whycon_pose', pid_error, queue_size=5)
    pub = rospy.Publisher('/cmd_vel', msg.Twist, queue_size=5)
    take_off_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)
    reset_pub = rospy.Publisher('/ardrone/reset', Empty, queue_size=5)

    marker_ids = marker_pose.get_marker_ids()

    ori_z = 0
    xyz = (0,0,0,0,0,0)
    th = 0
    status = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(doc_msg)
        # state dict for pid
        state = dict()
        state['lastError'] = np.array([0.,0.,0.,0.])

        # values of x and y may remain same
        if aruco_front:
            # xy_pid = [1, 0.0, 0.0]
            if aruco_mapping:
                xy_pid = [0.15, 0.0025, 0.025]
                state['p'] = np.array([xy_pid[0], xy_pid[0], 0.3, 1.0], dtype=float)
                state['i'] = np.array([xy_pid[1], xy_pid[1], 0.0025, 0.0], dtype=float)
                state['d'] = np.array([xy_pid[2], xy_pid[2], 0.15, 0.0], dtype=float)
            else:
                xy_pid = [0.3, 0.05, 0.4]
                state['p'] = np.array([xy_pid[0], xy_pid[0], 1, 0.6], dtype=float)
                state['i'] = np.array([xy_pid[1], xy_pid[1], 0.1, 0.1], dtype=float)
                state['d'] = np.array([xy_pid[2], xy_pid[2], 1, 0.05], dtype=float)
            # state['p'] = np.array([xy_pid[0], xy_pid[0], 1, 0.6], dtype=float)
            # state['i'] = np.array([xy_pid[1], xy_pid[1], 0., 0.], dtype=float)
            # state['d'] = np.array([xy_pid[2], xy_pid[2], 0, 0.], dtype=float)
        else:
            xy_pid_bottom = [2, 0., 0.2]
            state['p'] = np.array([xy_pid_bottom[0], xy_pid_bottom[0], 1, 0.5], dtype=float)
            state['i'] = np.array([xy_pid_bottom[1], xy_pid_bottom[1], 0.1, 0.1], dtype=float)
            state['d'] = np.array([xy_pid_bottom[2], xy_pid_bottom[2], 1, 0.05], dtype=float)

        state['integral'] = np.array([0.,0.,0.,0.])
        state['derivative'] = np.array([0.,0.,0.,0.])

        twist = msg.Twist()
        import time
        while 1:
            state['last_time'] = time.time()
            key = getKey()
            if key in moveBindings.keys():
                xyz = moveBindings[key]
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
                while 1:
                    current_pose = global_pose.as_waypoints()
                    pid_twist, state = pid(current_pose, state, aruco_front)
                    pub.publish(pid_twist)

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
                    elif key == ' ':
                        reset_pub.publish()
                        break
                    
            elif key == 'p':
                last_twist = np.zeros(4)
                marker_not_detected_count = 0
                while 1:
                    if aruco_mapping:
                        set_array = marker_pose.as_waypoints()
                        set_array[0] += 1.5
                        current_pose = global_pose.as_waypoints()

                        pid_twist, state = pid(current_pose, state, aruco_front, set_array)

                        if (current_pose == np.array([0., 0., 0., 0.])).all():
                            marker_not_detected_count += 1
                            
                        if (last_twist == np.array([pid_twist.linear.x, pid_twist.linear.y, pid_twist.linear.z, pid_twist.angular.z])).all():
                            marker_not_detected_count += 1

                        if marker_not_detected_count > 2:
                            pub.publish(twist)
                            marker_not_detected_count = 0
                            print('feed stuck!!!')
                        else:
                            pub.publish(pid_twist)
                            
                        last_twist[0] = pid_twist.linear.x
                        last_twist[1] = pid_twist.linear.y
                        last_twist[2] = pid_twist.linear.z
                        last_twist[3] = pid_twist.angular.z
                    else:
                        current_pose = global_pose.as_waypoints()
                        pid_twist, state = pid(current_pose, [-1, 0, 0, 0], state)
                        pub.publish(pid_twist)

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
                    elif key == ' ':
                        reset_pub.publish()
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
            elif (key == '\x03'):
                    break

            # xyz = np.clip(np.array(xyz), -0.3, 0.3)
            twist.linear.x = xyz[0]
            twist.linear.y = xyz[1]
            twist.linear.z = xyz[2]
            twist.angular.x = xyz[3]
            twist.angular.y = xyz[4]
            twist.angular.z = xyz[5]
            pub.publish(twist)

    # except Exception as e:
    #     print e

    finally:
        twist = msg.Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        land_pub.publish()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

