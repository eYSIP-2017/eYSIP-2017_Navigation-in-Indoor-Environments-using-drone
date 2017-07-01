#!/usr/bin/env python  
import rospy

# Because of transformations
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
    rospy.init_node('transform_handler')
    real_drone = bool(rospy.get_param('~real_drone', 'false'))
    if real_drone:
        drone_world = 'odom'
    else:
        drone_world = 'nav'

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    found = False
    while not rospy.is_shutdown():
        try:
            trans1 = tfBuffer.lookup_transform('camera_position', 'world', rospy.Time())
            trans2 = tfBuffer.lookup_transform(drone_world, 'ardrone_base_frontcam', rospy.Time())

            trans = multiply_transforms(trans2.transform, trans1.transform)
            if not found:
                static_trans = trans
                found = True
            static_trans.header.stamp = rospy.Time.now()
            static_trans.header.frame_id = drone_world
            static_trans.child_frame_id = 'world'
            print(static_trans)
            br = tf2_ros.StaticTransformBroadcaster()
            br.sendTransform(static_trans)

            # print(trans1.asMatrix(trans1.translation, trans1.rotation))
            # trans
            # print(trans1)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print('not yet found')
            continue
        rate.sleep()
