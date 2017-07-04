#!/usr/bin/env python
"""Generate transform between drone's odom and aruco's world."""
import rospy

# Because of transformations
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
import numpy as np
from pyquaternion import Quaternion


def as_transformation_matrix(trans):
    """Convert geomety_msgs.msg.TransformStamped to Transformation matrix.

    Args:
        trans (geomety_msgs.msg.TransformStamped): transform to be converted.

    Returns:
        numpy.array: transformation matrix generated.
    """
    mat = Quaternion(
        trans.rotation.w,
        trans.rotation.x,
        trans.rotation.y,
        trans.rotation.z)
    mat = mat.transformation_matrix
    mat[0][3] = trans.translation.x
    mat[1][3] = trans.translation.y
    mat[2][3] = trans.translation.z
    return mat


def as_transformation_ros(mat):
    """Convert Transformation matrix to geomety_msgs.msg.TransformStamped.

    Args:
        mat (numpy.array): transformation matrix to be converted.

    Returns:
        geomety_msgs.msg.TransformStamped: transform generated.
    """
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
    """Multiply 2 transforms.

    Args:
        trans1 (geomety_msgs.msg.TransformStamped): first transform to be multiplied.
        trans2 (geomety_msgs.msg.TransformStamped): second transform to be multiplied.

    Returns:
        geomety_msgs.msg.TransformStamped: final transform.
    """
    mat1 = as_transformation_matrix(trans1)
    mat2 = as_transformation_matrix(trans2)
    return as_transformation_ros(np.dot(mat1, mat2))


if __name__ == '__main__':
    rospy.init_node('transform_handler')
    real_drone = bool(rospy.get_param('~real_drone', 'false'))
    # check to see if you are working with read drone or in simulation
    # the world frame is different for both
    if real_drone:
        drone_world = 'odom'
    else:
        drone_world = 'nav'

    # initialising tf listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # setting up rate at which to transmit data
    rate = rospy.Rate(10.0)
    found = False
    # stay in loop unless rospy is closed
    while not rospy.is_shutdown():
        try:
            trans1 = tfBuffer.lookup_transform(
                'camera_position', 'world', rospy.Time())
            trans2 = tfBuffer.lookup_transform(
                drone_world, 'ardrone_base_frontcam', rospy.Time())

            # get the transform fro drone_world to aruco's world
            trans = multiply_transforms(trans2.transform, trans1.transform)
            # check to ensure that only the first transform is considered
            if not found:
                static_trans = trans
                found = True

            # add frames to the transform
            static_trans.header.stamp = rospy.Time.now()
            static_trans.header.frame_id = drone_world
            static_trans.child_frame_id = 'world'
            print(static_trans)
            # Publish a static transform
            br = tf2_ros.StaticTransformBroadcaster()
            br.sendTransform(static_trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('not yet found')
            continue
        # sleep for the time in rate
        rate.sleep()
