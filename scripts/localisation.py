#!/usr/bin/env python
from __future__ import print_function

import rospy

from geometry_msgs import msg
from tf.transformations import euler_from_quaternion
from ardrone_autonomy.msg import Navdata
from aruco_mapping.msg import ArucoMarker
# from tf2_msgs.msg import TFMessage
import tf

from time import time
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
    # print(ekf.x.state[1])

def aruco_callback(aru):
    # print(tf_data.transforms[0].child_frame_id)
    # if tf_data.transforms[0].child_frame_id == 'camera_position' and tf_data.transforms[0].header.frame_id == 'nav':
    #     print(tf_data.transforms[0])
    # while True:
    #     pass
    # print(type(tf_data.transforms))
    # for transform in tf_data.transforms:


    trans = None
    while trans is None:
        try:
            trans, rot = tf_listener.lookupTransform('camera_position', 'nav', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            continue
    euler = euler_from_quaternion(rot)

    ekf.x.observe_pose(trans[0], ekf.var_pose_observation_xy)
    ekf.y.observe_pose(trans[1], ekf.var_pose_observation_xy)
    ekf.z.observe_pose(trans[2], ekf.var_pose_observation_z_aruco)

    # check if eular 0 is correct!!!!!!
    ekf.roll.observe(np.degrees(euler[0]), ekf.var_pose_observation_rp_aruco)
    ekf.pitch.observe(np.degrees(euler[2]), ekf.var_pose_observation_rp_aruco)

    ekf.yaw.observe_pose(np.degrees(euler[1]), ekf.var_pose_observation_yaw)

def make_prediction(active_control):
    current_time = time()
    dt = current_time - make_prediction.previous_time
    ekf.prediction(active_control, dt)
    make_prediction.previous_time = current_time
    kalman_pose_pub.publish(ekf.get_current_pose())

if __name__ == "__main__":
    rospy.init_node('localisation')
    ekf = extendedKalmanFilter()
    tf_listener = tf.TransformListener()

    # aruco_front = bool(rospy.get_param('~aruco_front', 'true'))
    # rospy.Subscriber("/ardrone/navdata", Navdata, navdata_callback)
    rospy.Subscriber('/aruco_poses', ArucoMarker, aruco_callback)
    make_prediction.previous_time = time()
    rospy.Subscriber('/cmd_vel', msg.Twist, make_prediction)
    
    kalman_pose_pub = rospy.Publisher('/kalman_pose', msg.Pose, queue_size=1)
    rospy.spin()