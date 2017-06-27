#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion
from drone_application.msg import pid_error
from pose import Pose
import numpy as np
import pickle

def compute_distances(all_poses):
    # closer = all_poses[0]
    # farther = all_poses[1]
    # if closer[0] > farther[0]:
        # closer, farther = farther, closer
    # temp = (farther[0] * np.cos(farther[3])) - (closer[0] * np.cos(closer[3]))
    # temp = np.sqrt(np.sum(np.square(farther[1:3] - closer[1:3])))
    # temp = temp / np.sin(farther[3])
    temp = np.sqrt(np.sum(np.square(all_poses[1] - all_poses[0])))
    print(temp)

def predict_group(all_poses):
    all_poses.sort(key=lambda x: x[0])
    # print(all_poses)
    pred = np.zeros(6)
    
    pred[0] = np.sqrt(np.sum(np.square(all_poses[0] - all_poses[1])))
    pred[1] = np.sqrt(np.sum(np.square(all_poses[0] - all_poses[2])))
    pred[2] = np.sqrt(np.sum(np.square(all_poses[2] - all_poses[1])))
    pred[3] = np.arccos((pred[0] + pred[1] - pred[2]) / (2 * pred[0] * pred[1]))
    pred[4] = np.arccos((pred[1] + pred[2] - pred[0]) / (2 * pred[1] * pred[2]))
    pred[5] = np.arccos((pred[2] + pred[0] - pred[1]) / (2 * pred[2] * pred[0]))

    print(nn.predict([pred]))


def get_pose_from_whycon(whycon_poses):
    global all_poses
    # x - +ve left of camera
    # y - +ve below of camera
    # z - higher away from camera
    # yaw - euler[1] - 1.57 at centre decreasing towards edges
    i = 0
    for whycon_pose in whycon_poses.poses:
        temp_pose.convert_geometry_transform_to_pose(whycon_pose, remap=['z', 'x', 'y', 1])
        temp_pose.x *= -1
        all_poses.append(temp_pose.just_xy())
        x = temp_pose.as_waypoints()
        x = np.append(x, i)
        pose_pub.publish(x)
        i += 1

    # compute_distances(all_poses)
    predict_group(all_poses)
    all_poses = list()

if __name__ == '__main__':
    rospy.init_node('detect_whycon')
    temp_pose = Pose()
    all_poses = list()
    nn = pickle.load(open(r'/home/ros/whycon_ws/src/eYSIP-2017_Navigation-in-Indoor-Environments-using-drone/whycon_formation_data/trained_nn.p', 'rb' ))
    rospy.Subscriber('/whycon/poses', PoseArray, get_pose_from_whycon)
    pose_pub = rospy.Publisher('whycon_pose', pid_error, queue_size=5)
    rospy.spin()
