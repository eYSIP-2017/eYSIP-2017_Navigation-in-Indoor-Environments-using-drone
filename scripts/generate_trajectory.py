#! /usr/bin/env python

import rospy

import actionlib

import drone_application.msg

class moveAction(object):
    # create messages that are used to publish feedback/result
    _feedback = drone_application.msg.moveFeedback()
    _result = drone_application.msg.moveResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, drone_application.msg.moveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print('server ready')
      
    def execute_cb(self, goal):
        # helper variables
        success = True
        
        # append the seeds for the fibonacci sequence
        self._feedback.difference = [1]
        
        # publish info to the console for the user
        print(type(self._feedback.difference))
        # rospy.loginfo('giving feedback', self._feedback.difference)
        
        # start executing the action
        print(goal.waypoint)
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
    rospy.init_node('move_to_waypoint')
    server = moveAction(rospy.get_name())
    print(rospy.get_name())
    rospy.spin()