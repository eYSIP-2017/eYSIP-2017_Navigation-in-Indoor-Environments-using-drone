#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import drone_application.msg
from aruco_mapping.msg import ArucoMarker
import tf
from geometry_msgs.msg import Twist
import numpy as np
from pid import pid
import time
from pose import Pose

def get_camera_pose(temp_pose):
    camera_pose.convert_geometry_transform_to_pose(temp_pose.global_camera_pose)

class moveAction(object):
    # create messages that are used to publish feedback/result
    _feedback = drone_application.msg.moveFeedback()
    _result = drone_application.msg.moveResult()

    def __init__(self, name):
        self._action_name = name
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self._as = actionlib.SimpleActionServer(self._action_name, drone_application.msg.moveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print('server ready')

    # returns the current pose of the drone
    def moniter_transform(self):
        trans = None
        while trans is None:
            try:
                # trans, rot = self.tf_listener.lookupTransform('/nav', '/base_link', rospy.Time(0))
                trans, rot = self.tf_listener.lookupTransform('/ardrone_base_link', '/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        euler = tf.transformations.euler_from_quaternion(rot)
        return np.array([trans[0], trans[1], trans[2], euler[2]])

    def move_to_waypoint(self, waypoint):
        current_pose = self.moniter_transform()
        error_tolerence = np.array([0.1, 0.1, 0.1, 0.1])

        state = dict()
        state['lastError'] = np.array([0.,0.,0.,0.])
        state['integral'] = np.array([0.,0.,0.,0.])
        state['derivative'] = np.array([0.,0.,0.,0.])

        xy_pid = [0.15, 0.0025, 0.025]
        state['p'] = np.array([xy_pid[0], xy_pid[0], 0.3, 1.0], dtype=float)
        state['i'] = np.array([xy_pid[1], xy_pid[1], 0.0025, 0.0], dtype=float)
        state['d'] = np.array([xy_pid[2], xy_pid[2], 0.15, 0.0], dtype=float)

        state['last_time'] = time.time()
        i = 0
        # while (current_pose > waypoint + error_tolerence).all() and (current_pose < waypoint - error_tolerence).all():
        while (current_pose[i] > waypoint[i] + error_tolerence[i] or current_pose[i] < waypoint[i] - error_tolerence[i]):
        # while (current_pose[0] > waypoint[0] + error_tolerence[0] or current_pose[0] < waypoint[0] - error_tolerence[0]) or (current_pose[1] > waypoint[1] + error_tolerence[1] or current_pose[1] < waypoint[1] - error_tolerence[1]):
            pid_twist, state = pid(current_pose, waypoint, state)
            self.pub.publish(pid_twist)

            current_pose = self.moniter_transform()
            print(current_pose)
            # publish the feedback
            self._feedback.difference = waypoint - current_pose
            self._as.publish_feedback(self._feedback)


      
    def execute_cb(self, goal):
        # helper variables
        success = True
        
        # start executing the action
        print('starting exec')
        self.move_to_waypoint(np.array(goal.waypoint))
        twist = Twist()
        self.pub.publish(twist)
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
          
        if success:
            self._result.error = [2]
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_waypoint')
        camera_pose = Pose()
        rospy.Subscriber("/aruco_poses", ArucoMarker, get_camera_pose)
        server = moveAction(rospy.get_name())
        # rospy.Subscriber("/", , get_waypoints)

        rospy.spin()
    except rospy.ROSInterruptException:
        print('got exception')
    finally:
        twist = Twist()
        server.pub()
