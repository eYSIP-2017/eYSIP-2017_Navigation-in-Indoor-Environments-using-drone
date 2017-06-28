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
    pred = np.zeros(12)
    
    pred[0] = np.sqrt(np.sum(np.square(all_poses[0] - all_poses[1])))
    pred[1] = np.sqrt(np.sum(np.square(all_poses[0] - all_poses[2])))
    pred[2] = np.sqrt(np.sum(np.square(all_poses[2] - all_poses[1])))
    pred[3] = np.arccos((pred[0] + pred[2] - pred[1]) / (2 * pred[0] * pred[2]))
    pred[4] = np.arccos((pred[1] + pred[0] - pred[2]) / (2 * pred[1] * pred[0]))
    pred[5] = np.arccos((pred[2] + pred[1] - pred[0]) / (2 * pred[2] * pred[1]))

    centroid_y = (all_poses[0][0] + all_poses[1][0] + all_poses[2][0])/3
    centroid_z = (all_poses[0][1] + all_poses[1][1] + all_poses[2][1])/3
    print(np.around(np.array([centroid_y, centroid_z]), decimals=2))

    pred[6] = np.sqrt(np.square(all_poses[0][0] - centroid_y) + np.square(all_poses[0][1] - centroid_z))
    pred[7] = np.sqrt(np.square(all_poses[1][0] - centroid_y) + np.square(all_poses[1][1] - centroid_z))
    pred[8] = np.sqrt(np.square(all_poses[2][0] - centroid_y) + np.square(all_poses[2][1] - centroid_z))

    # get the absolute slope
    pred[9] = np.arctan(abs((all_poses[0][1] - all_poses[1][1]) / (all_poses[0][0] - all_poses[1][0])))
    pred[10] = np.arctan(abs((all_poses[1][1] - all_poses[2][1]) / (all_poses[1][0] - all_poses[2][0])))
    pred[11] = np.arctan(abs((all_poses[2][1] - all_poses[0][1]) / (all_poses[2][0] - all_poses[0][0])))
    # print(pred)
    # pred = obj_list[1].transform([pred])
    # print(obj_list[0].predict(pred))
    print(clf.predict([pred]))


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
    # obj_list = pickle.load(open(r'/home/ros/whycon_ws/src/eYSIP-2017_Navigation-in-Indoor-Environments-using-drone/whycon_formation_data/all_train.p', 'rb' ))
    clf = pickle.load(open(r'/home/ros/whycon_ws/src/eYSIP-2017_Navigation-in-Indoor-Environments-using-drone/whycon_formation_data/decision_train.p', 'rb' ))
    rospy.Subscriber('/whycon/poses', PoseArray, get_pose_from_whycon)
    pose_pub = rospy.Publisher('whycon_pose', pid_error, queue_size=5)
    rospy.spin()
