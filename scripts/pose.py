#!/usr/bin/env python
from __future__ import print_function
from tf.transformations import euler_from_quaternion
import numpy as np

class pose(object):
    def __init__(self, x=0, y=0, z=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.marker_ids = list()
        self.current_marker_id = None

    def convert_geometry_transform_to_pose(self, transform):
        try:
            euler = euler_from_quaternion((transform.rotation.x,
                                            transform.rotation.y,
                                            transform.rotation.z,
                                            transform.rotation.w
                                            ))
            self.x = transform.translation.x
            self.y = transform.translation.y
            self.z = transform.translation.z
        except AttributeError:
            euler = euler_from_quaternion((transform.orientation.x,
                                            transform.orientation.y,
                                            transform.orientation.z,
                                            transform.orientation.w
                                            ))
            self.z = transform.position.x
            self.x = -transform.position.y
            self.y = -transform.position.z
        self.yaw = -euler[0]

    def as_waypoints(self):
        return np.around(np.array([self.x, self.y, self.z, self.yaw]), decimals=3)

    def store_marker_ids(self, marker_ids):
       self.marker_ids = marker_ids
       
    def return_marker_ids(self):
       return self.marker_ids

    def store_current_marker_id(self, current_marker_id):
       self.current_marker_id = current_marker_id
       
    def return_current_marker_id(self):
       return self.current_marker_id