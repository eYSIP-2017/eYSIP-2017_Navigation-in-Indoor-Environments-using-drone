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

def get_waypoints(data):
    global waypoints, done_waypoints

    points_list = data.trajectory[0].multi_dof_joint_trajectory.points
    p = Pose()

    for transform in points_list:
        p.convert_geometry_transform_to_pose(transform.transforms[0])
        waypoints.append(list(p.as_waypoints()))
        # waypoints[-1][2] = waypoints[-1][2] + 3
    done_waypoints = True

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
        rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, get_waypoints)

        # waypoints = [[0,0,4,-3*np.pi/4], [1, 3, 2,-3*np.pi/4], [2,-1,4,3*np.pi/4], [3,0,4,3*np.pi/4]]
        waypoints = [[0,0,1,0], [1,0,1,0], [2,0,1,0]]

        # while not done_waypoints:
        #     pass
        generate_trajectory(waypoints)

        

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
        print('got exception')
