#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
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
                trans = tfBuffer.lookup_transform('camera_position', 'nav', rospy.Time())
                print(trans)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                continue

    for transform in points_list:
        if aruco_coords:
            p.convert_geometry_transform_to_pose(th.multiply_transforms(trans.transform, transform.transforms[0]).transform, ['z', 'x', 'y', 1])
        else:
            p.convert_geometry_transform_to_pose(transform.transforms[0])
        a = np.around(p.as_waypoints(), decimals=2)
        print(a)
        waypoints.append(list(a))
        # waypoints[-1][2] = waypoints[-1][2] + 3
    done_waypoints = True
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
        # waypoints = [[-1.5,0,0.8,0], [-1.5,-0.5,1,0], [-1.5,-1,1.1,0], [-1.5,-1.2,1.2,0], [-1.5,-1.5,1.3,0], [-1.5,-1.8,1.3,0], [-1.5,-2,1.3,0], [-1.5,-2.2,1,0], [-1.5,-2.4,0.9,0], [-1.5,-2.6,0.8,0], [-1.5,-2.6,0.5,0]]
        rospy.spin()
        # while not done_waypoints:
        #     pass
        # generate_trajectory(waypoints)
        # land_pub.publish()

        

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
