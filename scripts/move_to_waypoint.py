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
            self.empty_twist = Twist()
            self.camera_pose = Pose()
            print('server ready')
            print('Be careful, once a list of waypoints is given there is no way to stop execution yet.')

        # returns the current pose of the drone
        def moniter_transform(self):
            trans = None
            while trans is None:
                try:
                    if real_drone:
                        trans, rot = self.tf_listener.lookupTransform('/ardrone_base_link', '/odom', rospy.Time(0))
                    else:
                        trans, rot = self.tf_listener.lookupTransform('/base_link', 'nav', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            euler = tf.transformations.euler_from_quaternion(rot)
            return np.array([trans[0], trans[1], trans[2], euler[2]])

        def get_camera_pose(self, temp_pose):
            self.camera_pose.convert_geometry_transform_to_pose(temp_pose.global_camera_pose, ['z', 'y', 'x', 1])
            self.camera_pose.x *= -1

        def move_to_waypoint(self, waypoint):
            if aruco_mapping:
                current_pose = self.camera_pose.as_waypoints()
            else:
                current_pose = self.moniter_transform()

            state = dict()
            state['lastError'] = np.array([0.,0.,0.,0.])
            state['integral'] = np.array([0.,0.,0.,0.])
            state['derivative'] = np.array([0.,0.,0.,0.])

            # xy_pid = [0.3, 0.0, 0.25]
            xy_pid = [0.2, 0.00, 0.1]
            state['p'] = np.array([xy_pid[0], xy_pid[0], 0.3, 1.0], dtype=float)
            state['i'] = np.array([xy_pid[1], xy_pid[1], 0.0025, 0.0], dtype=float)
            state['d'] = np.array([xy_pid[2], xy_pid[2], 0.15, 0.0], dtype=float)

            state['last_time'] = time.time()
            last_twist = np.zeros(4)
            marker_not_detected_count = 0

            # not controlling yaw since drone was broken
            while not np.allclose(current_pose[0:-1], waypoint[0:-1], atol=0.1):
                if aruco_mapping:
                    current_pose = self.camera_pose.as_waypoints()
                    pid_twist, state = pid(current_pose, waypoint, state)
                    if (current_pose == np.array([0., 0., 0., 0.])).all():
                        marker_not_detected_count += 1
                        
                    if (last_twist == np.array([pid_twist.linear.x, pid_twist.linear.y, pid_twist.linear.z, pid_twist.angular.z])).all():
                        marker_not_detected_count += 1

                    if marker_not_detected_count > 2:
                        self.pub.publish(self.empty_twist)
                        marker_not_detected_count = 0
                        print('feed stuck!!!')
                    else:
                        self.pub.publish(pid_twist)

                    last_twist[0] = pid_twist.linear.x
                    last_twist[1] = pid_twist.linear.y
                    last_twist[2] = pid_twist.linear.z
                    last_twist[3] = pid_twist.angular.z
                else:
                    current_pose = self.moniter_transform()
                    pid_twist, state = pid(current_pose, waypoint, state)
                    self.pub.publish(pid_twist)
                
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
            self.pub.publish(self.empty_twist)
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
            real_drone = bool(rospy.get_param('~real_drone', 'false'))
            aruco_mapping = bool(rospy.get_param('~aruco_mapping', 'true'))

            server = moveAction(rospy.get_name())
            rospy.Subscriber("/aruco_poses", ArucoMarker, server.get_camera_pose)

            rospy.spin()
        except rospy.ROSInterruptException:
            print('got exception')
        finally:
            twist = Twist()
            server.pub()
