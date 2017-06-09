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

    def convert_geometry_transform_to_pose(self, transform):
        euler = euler_from_quaternion((transform.rotation.x,
                                        transform.rotation.y,
                                        transform.rotation.z,
                                        transform.rotation.w
                                        ))
        self.x = transform.translation.x
        self.y = transform.translation.y
        self.z = transform.translation.z
        self.yaw = euler[2]

    def as_waypoints(self):
        return np.around(np.array([self.x, self.y, self.z, self.yaw]), decimals=3)
