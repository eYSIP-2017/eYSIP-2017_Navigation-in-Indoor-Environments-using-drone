#!/usr/bin/env python
"""Extract, generate and send trajectory for excecution."""
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from moveit_msgs.msg import DisplayTrajectory
from tf.transformations import euler_from_quaternion
from collections import deque
import tf
import actionlib
import drone_application.msg
import numpy as np
from pose import Pose
import tf2_ros
import transform_handler as th

# Various publishers for controling the drone
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
take_off_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)


def send_trajectory(waypoints, client=None):
    """Send waypoints to follow.

    Sends individual waypoints as goals to moveaction server.

    Args:
        Client (SimpleActionClient): sent if client already created
    """
    if client is None:
        client = actionlib.SimpleActionClient(
            'move_to_waypoint', drone_application.msg.moveAction)
    for waypoint in waypoints:
        print(waypoint)
        print(type(waypoint))

        client.wait_for_server()
        print('server_found')

        goal = drone_application.msg.moveGoal(waypoint)
        client.send_goal(goal)

        client.wait_for_result()
        print(client.get_result())


waypoints = deque()
done_waypoints = False


def get_waypoints(data):
    """Extract the waypoints from move_group/display_planned_path.

    Very dirty implimentation, if a neater version is needed use
    the legacy_get_waypoints function.

    Note:
        If visualisation is needed aruco_coords should be true.
        If visualise_trajectory is true no momement will take place
        it will be stuck in a while loop which will exit only by exiting
        the module.

    Args:
        aruco_coords (bool): true if trajectory needed in aruco coords
        visualise_trajectory (bool): true if visualisaiton needed
    """
    global waypoints, done_waypoints

    points_list = data.trajectory[0].multi_dof_joint_trajectory.points
    p = Pose()
    if aruco_coords:
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        print('going to find tf')
        while True:
            try:
                if real_drone:
                    trans = tfBuffer.lookup_transform(
                        'world', 'odom', rospy.Time())
                else:
                    trans = tfBuffer.lookup_transform(
                        'world', 'nav', rospy.Time())
                print(trans)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('not yet found')
                continue
        i = 0
        br = []
        test_trans = []
        br1 = []
        test_trans1 = []

    for transform in points_list:
        if visualise_trajectory:
            test_trans.append(
                th.multiply_transforms(
                    trans.transform,
                    transform.transforms[0]))
            # create a transform stamped variable to store planned trajectory.
            trans_stamped = TransformStamped()
            trans_stamped.transform.translation.x = transform.transforms[0].translation.x
            trans_stamped.transform.translation.y = transform.transforms[0].translation.y
            trans_stamped.transform.translation.z = transform.transforms[0].translation.z
            trans_stamped.transform.rotation.x = transform.transforms[0].rotation.x
            trans_stamped.transform.rotation.y = transform.transforms[0].rotation.y
            trans_stamped.transform.rotation.z = transform.transforms[0].rotation.z
            trans_stamped.transform.rotation.w = transform.transforms[0].rotation.w
            trans_stamped.header.stamp = rospy.Time.now()
            trans_stamped.header.frame_id = 'odom'
            trans_stamped.child_frame_id = 'original_' + str(i)

            test_trans1.append(trans_stamped)
            br1.append(tf2_ros.StaticTransformBroadcaster())

            # add the frames to the trajectory in aruco's world frame
            test_trans[i].header.stamp = rospy.Time.now()
            test_trans[i].header.frame_id = 'world'
            test_trans[i].child_frame_id = 'test_transform_' + str(i)
            test_trans[i].transform.translation.y *= -1

            br.append(tf2_ros.StaticTransformBroadcaster())
            p.convert_geometry_transform_to_pose(test_trans[i].transform)

        if aruco_coords:
            # make the transform from odom/nav to world.
            temp_trans = th.multiply_transforms(
                trans.transform, transform.transforms[0])
            temp_trans.header.stamp = rospy.Time.now()
            temp_trans.header.frame_id = 'world'
            temp_trans.child_frame_id = 'test_transform_' + str(i)
            temp_trans.transform.translation.y *= -1
            p.convert_geometry_transform_to_pose(temp_trans)
            i += 1
        else:
            p.convert_geometry_transform_to_pose(transform.transforms[0])
        waypoints.append(np.around(p.as_waypoints(), decimals=2))
    done_waypoints = True
    if visualise_trajectory:
        # stay in this loop if visualisation in needed.
        while True:
            for i in range(len(test_trans)):
                br[i].sendTransform(test_trans[i])
                br1[i].sendTransform(test_trans1[i])

# IF THE ABOVE FUNCTION DOESNT WORK USE THIS


def legacy_get_waypoints(data):
    """Extract the waypoints from move_group/display_planned_path.

    Legacy function of get_waypoints, this is a way neater implimentation.

    Args:
        aruco_coords (bool): true if trajectory needed in aruco coords
        visualise_trajectory (bool): true if visualisaiton needed
    """
    global waypoints, done_waypoints

    points_list = data.trajectory[0].multi_dof_joint_trajectory.points
    p = Pose()

    for transform in points_list:
        # create a list of waypoints in pose.Pose after convertion from
        # planned trajectory
        p.convert_geometry_transform_to_pose(transform.transforms[0])
        waypoints.append(list(p.as_waypoints()))
    done_waypoints = True


if __name__ == '__main__':
    try:
        rospy.init_node('follow_trajectory', anonymous=True)
        real_drone = bool(rospy.get_param('~real_drone', 'false'))
        aruco_coords = bool(rospy.get_param('~aruco_coords', 'false'))
        visualise_trajectory = bool(rospy.get_param('~visualise_trajectory', 'false'))
        rospy.Subscriber(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            get_waypoints)

        # wait until waypoints are extracted from
        # move_group/display_planned_path
        while not done_waypoints:
            pass
        send_trajectory(waypoints)
        land_pub.publish()

    except rospy.ROSInterruptException:
        client = actionlib.SimpleActionClient(
            'move_to_waypoint', drone_application.msg.moveAction)
        client.wait_for_server()
        client.cancel_goal()
        print('got exception')
