#!/usr/bin/env python
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

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

take_off_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)

def generate_trajectory(waypoints, client=None):
    for waypoint in waypoints:
        print(waypoint)
        print(type(waypoint))
        result = send_goal(waypoint, client)
        print(result)
        # break

def send_goal(waypoint, client=None):
    if client is None:
        client = actionlib.SimpleActionClient('move_to_waypoint', drone_application.msg.moveAction)

    client.wait_for_server()
    print('server_found')

    goal = drone_application.msg.moveGoal(waypoint)
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()

waypoints = deque()
done_waypoints = False

def get_waypoints(data, aruco_coords=False):
    print('in')
    global waypoints, done_waypoints

    points_list = data.trajectory[0].multi_dof_joint_trajectory.points
    p = Pose()
    if aruco_coords:
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        print('going to find tf')
        while True:
            try:
                trans = tfBuffer.lookup_transform('world', 'odom', rospy.Time())
                print(trans)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                continue
    i = 0
    br = []
    test_trans = []
    br1 = []
    test_trans1 = []
    for transform in points_list:
        if aruco_coords:
            test_trans.append(th.multiply_transforms(trans.transform, transform.transforms[0]))
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

            test_trans[i].header.stamp = rospy.Time.now()
            test_trans[i].header.frame_id = 'world'
            test_trans[i].child_frame_id = 'test_transform_' + str(i)
            # test_trans[i].transform.translation.z, test_trans[i].transform.translation.y = test_trans[i].transform.translation.y, test_trans[i].transform.translation.z
            # test_trans[i].transform.translation.z, test_trans[i].transform.translation.x = test_trans[i].transform.translation.x, test_trans[i].transform.translation.z
            test_trans[i].transform.translation.y *= -1
            # test_trans[i].transform.translation.x *= -1


            # test_trans1[i].header.stamp = rospy.Time.now()
            # test_trans1[i].header.frame_id = 'odom'
            # test_trans1[i].child_frame_id = 'test_transform1_' + str(i)
            # print(test_trans)
            # print(i)
            br.append(tf2_ros.StaticTransformBroadcaster())
            # br[i].sendTransform(test_trans)
            p.convert_geometry_transform_to_pose(test_trans[i].transform)#, ['z', 'x', 'y', 1])
        else:
            p.convert_geometry_transform_to_pose(transform.transforms[0])
        a = np.around(p.as_waypoints(), decimals=2)
        print(a)
        waypoints.append(list(a))
        i += 1
        # waypoints[-1][2] = waypoints[-1][2] + 3
    done_waypoints = True
    while True:
        for i in range(len(test_trans)):
            br[i].sendTransform(test_trans[i])
            br1[i].sendTransform(test_trans1[i])
    # print(waypoints)

    # generate_trajectory(waypoints)

    # START STATE INFORMATION
    # print(data.trajectory_start.multi_dof_joint_state.transforms[0])
    # print(type(data.trajectory_start.multi_dof_joint_state.transforms[0]))

    # ALL THE WAYPOINT
    # print(data.trajectory[0].multi_dof_joint_trajectory.points[2].transforms[0])
    # print(len(data.trajectory[0].multi_dof_joint_trajectory.points))
    # print(type(data.trajectory[0].multi_dof_joint_trajectory.points[2].transforms[0]))

if __name__ == '__main__':
    try:
        rospy.init_node('follow_trajectory', anonymous=True)
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, get_waypoints, True)

        # waypoints = [[0,0,4,-3*np.pi/4], [1, 3, 2,-3*np.pi/4], [2,-1,4,3*np.pi/4], [3,0,4,3*np.pi/4]]
        # waypoints = [[1.5,0,1.2,0], [1.5,1,1.2,0], [1.5,2,1.2,0]]
        waypoints = [[-1.5,0,0.8,0], [-1.5,-0.5,1,0], [-1.5,-1,1.1,0], [-1.5,-1.2,1.2,0], [-1.5,-1.5,1.3,0], [-1.5,-1.8,1.3,0], [-1.5,-2,1.3,0], [-1.5,-2.2,1,0], [-1.5,-2.4,0.9,0], [-1.5,-2.6,0.8,0], [-1.5,-2.6,0.5,0]]
        # waypoints=[[ -1.5 ,-0.53 , 0.2  ,-0.12],
        #            # [ -1.5 ,-0.72 , 0.33 , 0.  ],
        #            [ -1.5 ,-0.92 , 0.46 , 0.12],
        #            # [ -1.5 ,-1.11 , 0.6  , 0.23],
        #            [ -1.5 ,-1.3  , 0.72 , 0.33],
        #            # [ -1.5 ,-1.46 , 0.82 , 0.4 ],
        #            [ -1.5 ,-1.61 , 0.91 , 0.47],
        #            # [ -1.5 ,-1.75 , 0.98 , 0.52],
        #            [ -1.5 ,-1.89 , 1.06 , 0.56],
        #            # [ -1.5 ,-2.   , 1.1  , 0.58],
        #            [ -1.5 ,-2.1  , 1.13 , 0.6 ],
        #            # [ -1.5 ,-2.18 , 1.14 , 0.6 ],
        #            [ -1.5 ,-2.26 , 1.14 , 0.61],
        #            # [ -1.5 ,-2.31 , 1.11 , 0.59],
        #            [ -1.5 ,-2.36 , 1.08 , 0.58],
        #            # [ -1.5 ,-2.4  , 1.04 , 0.56],
        #            [ -1.5 ,-2.45 , 0.99 , 0.53],
        #            # [ -1.5 ,-2.46 , 0.9  , 0.48],
        #            [ -1.5 ,-2.48 , 0.81 , 0.42],
        #            # [ -1.5 ,-2.5  , 0.71 , 0.35],
        #            [ -1.5 ,-2.52 , 0.62 , 0.28],
        #            # [ -1.5 ,-2.52 , 0.5  , 0.19],
        #            [ -1.5 ,-2.52 , 0.38 , 0.09],
        #            # [ -1.5 ,-2.53 , 0.26 ,-0.01],
        #            [ -1.5 ,-2.53 , 0.14 ,-0.12]]

        # rospy.spin()
        # while not done_waypoints:
        #     pass
        generate_trajectory(waypoints)
        land_pub.publish()

        

        # while not rospy.is_shutdown():
        #     trans, rot = moniter_transform(tf_listener)
        #     if trans is None:
        #         continue
        #     generate_trajectory(trans, rot, tf_listener)
        #     break
        # take_off_pub.publish()
        # print('published take off')
        # rospy.Subscriber("/ardrone/navdata", Navdata, moniter_navdata)
        # rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, callback)
        # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()
    except rospy.ROSInterruptException:
        client = actionlib.SimpleActionClient('move_to_waypoint', drone_application.msg.moveAction)
        client.wait_for_server()
        client.cancel_goal()
        print('got exception')
