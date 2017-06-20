#! /usr/bin/env python 

import rospy
from aruco_mapping.msg import *
from pose import pose
import numpy as np

# tran_x = 0
# tran_y = 0
# tran_z = 0
# ori_x = 0
# ori_y = 0
# ori_z = 0
# ori_w = o

def get_aruco_pose(temp_pose):
    # print(temp_pose.global_camera_pose.position.x)
    # print(type(temp_pose.global_marker_poses[1]))
    # print(temp_pose.marker_ids.index(current_marker_id))
    marker_pose.store_marker_ids(temp_pose.marker_ids)
    if marker_pose.get_current_marker_id() is not None:
        marker_pose.convert_geometry_transform_to_pose(temp_pose.global_marker_poses[temp_pose.marker_ids.index(marker_pose.get_current_marker_id())])

    global_pose.convert_geometry_transform_to_pose(temp_pose.global_camera_pose)
    print(global_pose.as_waypoints())

    


# tran_y = 0 
# def get_pose_from_next_aruco(data):
#     global tran_y
#     tran_y = data.pose.position.y
#     print(tran_y)

# rospy.init_node('marker_pose')
# rospy.Subscriber('aruco_markers', ArucoMarker, get_pose_from_next_aruco)

# def goto(current_marker_id):




if __name__ == '__main__':
    rospy.init_node('marker_pose')
    rospy.Subscriber('aruco_poses', ArucoMarker, get_aruco_pose)
    marker_pose = pose()
    global_pose = pose()

    max_found = False
    rospy.spin()

    # while(1):
    #     marker_ids = marker_pose.get_marker_ids()
    #     # min_found = False
    #     if len(marker_ids) != 0:
    #         if max_found == True:
    #             current_marker_id = min(marker_ids)

    #         else:
    #             current_marker_id = max(marker_ids)

    #         if current_marker_id == 201:
    #             max_found = True
    #         marker_pose.store_current_marker_id(current_marker_id)

    #     # goto(current_marker_id)


    #     # if min_found == True:
    #     #     current_marker_id = max(marker_ids)

    #     # else:
    #     #     current_marker_id = min(marker_ids)

    #     # if current_marker_id == 12:
    #     #     min_found = True

    #     set_array = marker_pose.as_waypoints()
    #     print(max_found, set_array, marker_pose.get_current_marker_id())