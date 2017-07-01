#!/usr/bin/env python
from __future__ import print_function
from tf.transformations import euler_from_quaternion
import numpy as np

class Pose(object):
    def __init__(self, x=0, y=0, z=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.marker_ids = list()
        self.current_marker_id = None
        self.max_found = False

    def __str__(self):
        return "x: {}    y: {}   z: {}   yaw: {}".format(self.x, self.y, self.z, self.yaw)

    def convert_geometry_transform_to_pose(self, transform, remap=['x', 'y', 'z', 1]):
        try:
            euler = euler_from_quaternion((transform.orientation.x,
                                            transform.orientation.y,
                                            transform.orientation.z,
                                            transform.orientation.w
                                            ))
            self.x = getattr(transform.position, remap[0])
            self.y = getattr(transform.position, remap[1])
            self.z = getattr(transform.position, remap[2])
            self.yaw = euler[remap[3]]

        except AttributeError:
            euler = euler_from_quaternion((transform.rotation.x,
                                            transform.rotation.y,
                                            transform.rotation.z,
                                            transform.rotation.w
                                            ))
            self.x = getattr(transform.translation, remap[0])
            self.y = getattr(transform.translation, remap[1])
            self.z = getattr(transform.translation, remap[2])
            self.yaw = euler[remap[3]]
                
            

    def as_waypoints(self):
        return np.around(np.array([self.x, self.y, self.z, self.yaw]), decimals=3)

    def just_xy(self):
        return np.around(np.array([self.y, self.z]), decimals=3)

    def store_marker_ids(self, marker_ids):
       self.marker_ids = marker_ids

    def get_marker_ids(self):
       return self.marker_ids

    def store_current_marker_id(self, current_marker_id):
       self.current_marker_id = current_marker_id

    def get_current_marker_id(self):
       return self.current_marker_id

    def get_max_found(self):
        return self.max_found

    def set_max_found(self, max_found):
        self.max_found = max_found