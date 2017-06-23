#!/usr/bin/env python
from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist, Pose, Vector3Stamped
from std_msgs.msg import Empty, Float64
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from ardrone_autonomy.msg import Navdata
from aruco_mapping.msg import ArucoMarker

from time import time
from pose import Pose
from pid import pid
import numpy as np
from kalman_filter import extendedKalmanFilter

def navdata_callback(nav):
    ekf.roll.observe(nav.rotX, ekf.var_pose_observation_rp_imu)
    ekf.pitch.observe(nav.rotY, ekf.var_pose_observation_rp_imu)
    # TODO: handle jump from -180 to 180
    ekf.yaw.observe_pose(nav.rotZ, 1*1)
    if ekf.yaw.prev is not None:
        ekf.yaw.observe_speed(nav.rotZ - ekf.yaw.prev, 1*1)
    ekf.yaw.prev = nav.rotZ

    yaw_rad = np.radians(ekf.yaw.state[0])
    vx_global = (np.sin(yaw_rad) * nav.vx + np.cos(yaw_rad) * nav.vy) / 1000.0
    vy_global = (np.cos(yaw_rad) * nav.vx - np.sin(yaw_rad) * nav.vy) / 1000.0

    ekf.x.observe_speed(vx_global, ekf.var_speed_observation_xy)
    ekf.y.observe_speed(vy_global, ekf.var_speed_observation_xy)
    ekf.z.observe_pose(nav.altd / 1000.0, ekf.var_pose_observation_z_IMU)
    if ekf.z.prev is not None:
        ekf.z.observe_speed(nav.altd/1000.0 - ekf.z.prev, ekf.var_pose_observation_z_IMU)
    ekf.z.prev = nav.altd / 1000.0

def aruco_callback(aru):
    euler = euler_from_quaternion((aru.global_camera_pose.orientation.x,
                                   aru.global_camera_pose.orientation.y,
                                   aru.global_camera_pose.orientation.z,
                                   aru.global_camera_pose.orientation.w
                                   ))
    ekf.x.observe_pose(aru.global_camera_pose.position.x, ekf.var_pose_observation_xy)
    ekf.y.observe_pose(aru.global_camera_pose.position.y, ekf.var_pose_observation_xy)
    ekf.x.observe_pose(aru.global_camera_pose.position.x, ekf.var_pose_observation_z_aruco)

    # check if eular 0 is correct!!!!!!
    ekf.roll.observe(np.degrees(euler[0]), ekf.var_pose_observation_rp_aruco)
    ekf.pitch.observe(np.degrees(euler[2]), ekf.var_pose_observation_rp_aruco)

    ekf.yaw.observe_pose(np.degrees(euler[1]), ekf.var_pose_observation_yaw)
    print('finished aruco')

def make_prediction(active_control):
    current_time = time()
    dt = current_time - make_prediction.previous_time
    ekf.prediction(active_control, dt)
    make_prediction.previous_time = current_time
    # print(dt)
    print(ekf)


if __name__ == "__main__":
    rospy.init_node('localisation')
    ekf = extendedKalmanFilter()

    # aruco_front = bool(rospy.get_param('~aruco_front', 'true'))
    rospy.Subscriber("/ardrone/navdata", Navdata, navdata_callback)
    rospy.Subscriber('/aruco_poses', ArucoMarker, aruco_callback)
    make_prediction.previous_time = time()
    rospy.Subscriber('/cmd_vel', Twist, make_prediction)
    
    drone_pose_pub = rospy.Publisher('/drone_pose', Float64, queue_size=1)
    rospy.spin()