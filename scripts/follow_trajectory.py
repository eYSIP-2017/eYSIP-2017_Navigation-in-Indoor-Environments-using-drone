#!/usr/bin/env python
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

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

take_off_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=5)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=5)

# twist = Twist()
# twist.linear.x = f[0]
# twist.linear.y = f[1]
# twist.angular.z

waypoints = [[0,0,0,0,0,0,1], [1,0,0,0,0,0,1], [2,0,0,0,0,0,1], [3,0,0,0,0,0,1]]

def moniter_transform(tf_listener):
    try:
        (trans,rot) = tf_listener.lookupTransform('/nav', '/base_link', rospy.Time(0))
        return trans, rot
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None, None

first_time = True

def generate_trajectory(trans, rot, tf_listener):
    global first_time
    start_state = list()
    if first_time:
        start_state = [trans[0], trans[1], trans[2], 0, 0, 0]
        first_time = False

    for waypoint in waypoints:
        # print(waypoint)
        euler = euler_from_quaternion((waypoint[3],
                                      waypoint[4],
                                      waypoint[5],
                                      waypoint[6]
                                      ))
        twist = Twist()
        print(start_state[0] + waypoint[0], trans[0])
        while (start_state[0] + waypoint[0]) > trans[0]:
            twist.linear.x = 1
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            pub.publish(twist)
            trans, rot = moniter_transform(tf_listener)
            print(start_state[0] + waypoint[0], trans[0])





def callback(data):
    pass
    # quaternion = (data.pose.orientation.x,
    #               data.pose.orientation.y,
    #               data.pose.orientation.z,
    #               data.pose.orientation.w
    #               )
    # euler = euler_from_quaternion(quaternion)

    # points_list = data.trajectory[0].multi_dof_joint_trajectory.points

    # waypoints = deque()
    # for transform in points_list:
    #     curr_waypoint = deque()
    #     transform.transforms[0]

    # START STATE INFORMATION
    # print(data.trajectory_start.multi_dof_joint_state.transforms[0])
    # print(type(data.trajectory_start.multi_dof_joint_state.transforms[0]))

    # ALL THE WAYPOINT
    # print(data.trajectory[0].multi_dof_joint_trajectory.points[2].transforms[0])
    # print(len(data.trajectory[0].multi_dof_joint_trajectory.points))
    # print(type(data.trajectory[0].multi_dof_joint_trajectory.points[2].transforms[0]))

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('follow_trajectory', anonymous=True)
    tf_listener = tf.TransformListener()

    client = actionlib.SimpleActionClient('move_to_waypoint', drone_application.msg.moveAction)

    client.wait_for_server()
    print('server_found')

    goal = drone_application.msg.moveGoal(waypoints[1])
    client.send_goal(goal)

    client.wait_for_result()
    print(client.get_result())
    return

    while not rospy.is_shutdown():
        trans, rot = moniter_transform(tf_listener)
        if trans is None:
            continue
        generate_trajectory(trans, rot, tf_listener)
        break
    # take_off_pub.publish()
    # print('published take off')
    # rospy.Subscriber("/ardrone/navdata", Navdata, moniter_navdata)
    # rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

# runs the listener function if the file is run as a script
if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        print('interrupt')
    finally:
        twist = Twist()
        pub.publish(twist)
