#!/usr/bin/env python  
import rospy

# Because of transformations
import tf

import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
import numpy as np
from pyquaternion import Quaternion

def as_transformation_matrix(trans):
    mat = Quaternion(trans.rotation.w, trans.rotation.x, trans.rotation.y, trans.rotation.z)
    mat = mat.transformation_matrix
    mat[0][3] = trans.translation.x
    mat[1][3] = trans.translation.y
    mat[2][3] = trans.translation.z
    return mat

def as_transformation_ros(mat):
    q = Quaternion(matrix=mat)
    trans = TransformStamped()
    trans.transform.translation.x = mat[0][3]
    trans.transform.translation.y = mat[1][3]
    trans.transform.translation.z = mat[2][3]
    trans.transform.rotation.x = q[1]
    trans.transform.rotation.y = q[2]
    trans.transform.rotation.z = q[3]
    trans.transform.rotation.w = q[0]
    return trans

def multiply_transforms(trans1, trans2):
    mat1 = as_transformation_matrix(trans1)
    mat2 = as_transformation_matrix(trans2)
    return as_transformation_ros(np.dot(mat1, mat2))


if __name__ == '__main__':
    rospy.init_node('transform_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            trans1 = tfBuffer.lookup_transform('camera_position', 'world', rospy.Time())
            trans2 = tfBuffer.lookup_transform('nav', 'ardrone_base_frontcam', rospy.Time())

            trans = multiply_transforms(trans2.transform, trans1.transform)
            trans.header.stamp = rospy.Time.now()
            trans.header.frame_id = 'nav'
            trans.child_frame_id = 'world'
            print(trans)
            br = tf2_ros.TransformBroadcaster()
            br.sendTransform(trans)

            # print(trans1.asMatrix(trans1.translation, trans1.rotation))
            # trans
            # print(trans1)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('not yet found')
            continue
