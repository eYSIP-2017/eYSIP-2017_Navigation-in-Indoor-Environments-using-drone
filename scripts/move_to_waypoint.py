#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import drone_application.msg
from aruco_mapping.msg import ArucoMarker
import tf
from geometry_msgs.msg import Twist
import numpy as np
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
                trans, rot = self.tf_listener.lookupTransform('/nav', '/base_link', rospy.Time(0))
                # trans, rot = self.tf_listener.lookupTransform('/odom', '/ardrone_base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        euler = tf.transformations.euler_from_quaternion(rot)
        return np.array([trans[0], trans[1], trans[2], euler[2]])

    def move_to_waypoint(self, waypoint):
        current_pose = self.moniter_transform()

        end_pose = waypoint #- current_pose

        twist = Twist()
        error_tolerence = np.array([0.1, 0.1, 0.1, 0.1])
        while (current_pose > end_pose + error_tolerence).any() or (current_pose < end_pose - error_tolerence).any():
            difference = end_pose - current_pose
            # difference = difference.astype(int)
            # difference = np.around(difference, decimals=2)
            a = (difference[0] * np.cos(end_pose[3])) + (difference[1] * np.sin(end_pose[3]))
            difference[1] = (difference[1] * np.cos(end_pose[3])) - (difference[0] * np.sin(end_pose[3]))
            difference[0] = a
            for_twist = np.clip(difference, -1, 1)
            twist.linear.x = for_twist[0]
            twist.linear.y = for_twist[1]
            twist.linear.z = for_twist[2]
            twist.angular.z = for_twist[3]
            self.pub.publish(twist)
            current_pose = self.moniter_transform()
            print(current_pose)
            # publish the feedback
            self._feedback.difference = difference
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
