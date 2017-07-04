#!/usr/bin/env python

"""Handle Control of drone using keyboard."""
from __future__ import print_function

import rospy

from geometry_msgs import msg
from std_msgs.msg import Empty, Float64
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from ardrone_autonomy.msg import Navdata
from aruco_mapping.msg import *
from pose import Pose
import actionlib

import sys
import select
import termios
import tty
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
    'i': (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    'k': (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # left and right
    'j': (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
    'l': (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
    # up and down
    'y': (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    'h': (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
    # counterclockwise and clockwise
    'u': (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    'o': (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
    # stop
    'm': ((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
}


def getKey():
    """Get key from keyboard.

    Returns:
        string: The key pressed
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def check_battery(data):
    """Check the battery and lands if low."""
    if data.batteryPercent < 15:
        land_pub.publish()


def get_pose_from_aruco(temp_pose):
    """Callback for pose using aruco.

    Depending on the value of aruco_mapping this function will
    store the global_camera_pose using aruco_mapping or
    store the camera pose using viewpoint estimation.

    modify global poses based on this convention:
        global_pose.x = x-axis - This is correct no need to change
        global_pose.y = y-axis (-ve when drone is on the left of the marker's origin)
        global_pose.z = z-axis (+ve when drone is abose the marker's origin)

    """
    # This functionality is for moving from one aruco to the next.
    # check if aruco_mapping is to be used
    if aruco_mapping:
        marker_pose.store_marker_ids(temp_pose.marker_ids)
        # there should be at least 1 marker detected
        if len(temp_pose.marker_ids) != 0:
            # if max found then find the lowest id else find the highest
            if marker_pose.get_max_found():
                current_marker_id = min(temp_pose.marker_ids)
                # if reached back to start land
                if current_marker_id == 13:
                    pub.publish(twist)
                    land_pub.publish()
            else:
                current_marker_id = max(temp_pose.marker_ids)
            # if largest value found set max_found to true
            if current_marker_id == 19:
                marker_pose.set_max_found(True)
        # ensure that the first marker is detected and there is a marker being detected now
        if marker_pose.get_current_marker_id() is not None and len(temp_pose.marker_ids) != 0:
            marker_pose.convert_geometry_transform_to_pose(
                temp_pose.global_marker_poses[temp_pose.marker_ids.index(current_marker_id)])
        # store global camera pose
        global_pose.convert_geometry_transform_to_pose(
            temp_pose.global_camera_pose)
    else:
        global_pose.convert_geometry_transform_to_pose(temp_pose.pose)


def get_pose_from_kalman(kalman_pose):
    """Get and store pose generated my EKF."""
    global_pose.convert_geometry_transform_to_pose(kalman_pose)


if __name__ == "__main__":
    # create pose objects to hold drone pose and marker pose
    marker_pose = Pose()
    global_pose = Pose()

    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('ardrone_teleop')
    # get commmand line args to handle working of node
    aruco_front = bool(rospy.get_param('~aruco_front', 'true'))
    aruco_mapping = bool(rospy.get_param('~aruco_mapping', 'true'))
    localisation = bool(rospy.get_param('~localisation', 'true'))

    # subscribe to navdata to check if battery is low and land
    rospy.Subscriber("/ardrone/navdata", Navdata, check_battery)
    # subscribing to aruco_mapping is essential for localisation
    if localisation:
        rospy.Subscriber('kalman_pose', msg.Pose, get_pose_from_kalman)
        aruco_mapping = True
    elif aruco_mapping:
        rospy.Subscriber('aruco_poses', ArucoMarker, get_pose_from_aruco)
    else:
        rospy.Subscriber("/Estimated_marker", Marker, get_pose_from_aruco)

    # initialising various publishers
    temp_pub = rospy.Publisher('/yaw', Float64, queue_size=5)
    pub = rospy.Publisher('/cmd_vel', msg.Twist, queue_size=5)
    take_off_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)
    reset_pub = rospy.Publisher('/ardrone/reset', Empty, queue_size=5)

    marker_ids = marker_pose.get_marker_ids()

    xyz = (0, 0, 0, 0, 0, 0)
    try:
        print(doc_msg)
        # state dict for pid
        state = dict()
        # initialising variables for PID
        state['lastError'] = np.array([0., 0., 0., 0.])

        # setting the PID gains
        # values of x and y may remain same
        # check if the aruco is in front or not
        if aruco_front:
            # just comment the inner values and use this for simulation
            xy_pid = [1, 0.0, 0.0]
            # check if aruco_mapping is being used or not
            if aruco_mapping:
                xy_pid = [0.15 / 2, 0.015 / 2, 0.025 / 2]
                state['p'] = np.array(
                    [xy_pid[0], xy_pid[0], 0.3 / 2, 1.0], dtype=float)
                state['i'] = np.array(
                    [xy_pid[1], xy_pid[1], 0.0025 / 2, 0.0], dtype=float)
                state['d'] = np.array(
                    [xy_pid[2], xy_pid[2], 0.15 / 2, 0.0], dtype=float)
            else:
                xy_pid = [0.3, 0.05, 0.4]
                state['p'] = np.array(
                    [xy_pid[0], xy_pid[0], 1, 0.6], dtype=float)
                state['i'] = np.array(
                    [xy_pid[1], xy_pid[1], 0.1, 0.1], dtype=float)
                state['d'] = np.array(
                    [xy_pid[2], xy_pid[2], 1, 0.05], dtype=float)
            # state['p'] = np.array([xy_pid[0], xy_pid[0], 1, 0.6], dtype=float)
            # state['i'] = np.array([xy_pid[1], xy_pid[1], 0., 0.], dtype=float)
            # state['d'] = np.array([xy_pid[2], xy_pid[2], 0, 0.], dtype=float)
        else:
            xy_pid_bottom = [2, 0., 0.2]
            state['p'] = np.array(
                [xy_pid_bottom[0], xy_pid_bottom[0], 1, 0.5], dtype=float)
            state['i'] = np.array(
                [xy_pid_bottom[1], xy_pid_bottom[1], 0.1, 0.1], dtype=float)
            state['d'] = np.array(
                [xy_pid_bottom[2], xy_pid_bottom[2], 1, 0.05], dtype=float)

        # initialising variables for PID
        state['integral'] = np.array([0., 0., 0., 0.])
        state['derivative'] = np.array([0., 0., 0., 0.])

        twist = msg.Twist()
        import time
        # execute the following commands until node is shutdown.
        while True:
            state['last_time'] = time.time()
            key = getKey()
            # check the predifined key bindings
            if key in moveBindings.keys():
                xyz = moveBindings[key]
            elif key == 't':
                # take off
                take_off_pub.publish()
            elif key == 'g':
                # land
                land_pub.publish()
            elif key == 'p':
                # start PID
                # initialisation of variables needed
                last_twist = np.zeros(4)
                marker_not_detected_count = 0
                # continue PID indefinatly until stopped of node exits
                while True:
                    # check if using aruco_mapping.
                    if aruco_mapping:
                        # get the marker to be the new set array
                        set_array = marker_pose.as_waypoints()
                        # offest x axis by 1.5 to maintain 1.5m distance
                        # from drone.
                        set_array[0] += 1.5
                        # get current pose
                        current_pose = global_pose.as_waypoints()

                        # run one iteration of PID
                        pid_twist, state = pid(
                            current_pose, set_array, state)

                        # if no aruco is being detected
                        if (current_pose == np.array([0., 0., 0., 0.])).all():
                            marker_not_detected_count += 1

                        # if the feed is stuck
                        if (last_twist == np.array(
                                [pid_twist.linear.x, pid_twist.linear.y, pid_twist.linear.z, pid_twist.angular.z])).all():
                            marker_not_detected_count += 1

                        # if we are sure it is stuck or no aruco found
                        if marker_not_detected_count > 2:
                            pub.publish(twist)
                            marker_not_detected_count = 0
                            print('feed stuck!!!')
                        else:
                            pub.publish(pid_twist)

                        # store last twist values to see if the feed is stuck
                        last_twist[0] = pid_twist.linear.x
                        last_twist[1] = pid_twist.linear.y
                        last_twist[2] = pid_twist.linear.z
                        last_twist[3] = pid_twist.angular.z
                    else:
                        current_pose = global_pose.as_waypoints()
                        pid_twist, state = pid(
                            current_pose, np.array([1.5, 0, 0, 0]), state)
                        pub.publish(pid_twist)

                    key = getKey()
                    if key == 's':
                        # Stop PID.
                        state['lastError'] = np.array([0., 0., 0., 0.])
                        state['integral'] = np.array([0., 0., 0., 0.])
                        state['derivative'] = np.array([0., 0., 0., 0.])
                        xyz = (0, 0, 0, 0, 0, 0)
                        break
                    elif key == 'f':
                        # print pid gains
                        print(
                            'yaw: {}, {}, {}; z-axis: {}, {}, {}; xy-axis: {}, {}, {};'.format(
                                state['p'][3],
                                state['i'][3],
                                state['d'][3],
                                state['p'][2],
                                state['i'][2],
                                state['d'][2],
                                state['p'][0],
                                state['i'][0],
                                state['d'][0]))
                        print('e - yaw;     d - z_axis;     c - xy_axis')
                    elif key == 'g':
                        land_pub.publish()
                        xyz = (0, 0, 0, 0, 0, 0)
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
            elif key == ' ':
                reset_pub.publish()
            elif key == 'f':
                print(
                    'yaw: {}, {}, {}; z-axis: {}, {}, {}; xy-axis: {}, {}, {};'.format(
                        state['p'][3],
                        state['i'][3],
                        state['d'][3],
                        state['p'][2],
                        state['i'][2],
                        state['d'][2],
                        state['p'][0],
                        state['i'][0],
                        state['d'][0]))
                print('e - yaw;     d - z_axis;     c - xy_axis')
            elif (key == '\x03'):
                break

            # Do not allow publishing twist values larget than 0.3
            xyz = np.clip(np.array(xyz), -0.3, 0.3)
            twist.linear.x = xyz[0]
            twist.linear.y = xyz[1]
            twist.linear.z = xyz[2]
            twist.angular.x = xyz[3]
            twist.angular.y = xyz[4]
            twist.angular.z = xyz[5]
            pub.publish(twist)
    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
        land_pub.publish()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
