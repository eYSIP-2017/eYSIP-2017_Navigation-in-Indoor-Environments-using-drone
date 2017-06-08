#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import drone_application.msg
import tf
import numpy as np

class moveAction(object):
    # create messages that are used to publish feedback/result
    _feedback = drone_application.msg.moveFeedback()
    _result = drone_application.msg.moveResult()

    def __init__(self, name):
        self._action_name = name
        self.tf_listener = tf.TransformListener()
        self._as = actionlib.SimpleActionServer(self._action_name, drone_application.msg.moveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print('server ready')

    def moniter_transform(self):
        trans = None
        while trans is None:
            try:
                trans, rot = self.tf_listener.lookupTransform('/nav', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return trans, rot

    def move_to_waypoint(self, waypoint):
        trans, rot = self.moniter_transform()
        euler = tf.transformations.euler_from_quaternion(rot)

        start_state = np.array([trans[0], trans[1], trans[2], euler[2]])
        # print(start_state)

      
    def execute_cb(self, goal):
        # helper variables
        success = True
        
        self._feedback.difference = [1]
        
        # start executing the action
        self.move_to_waypoint(goal.waypoint)
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        
        # publish the feedback
        self._as.publish_feedback(self._feedback)
          
        if success:
            self._result.error = [2]
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_waypoint')
        server = moveAction(rospy.get_name())

        rospy.spin()
    except rospy.ROSInterruptException:
        print('got exception')
