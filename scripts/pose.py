#!/usr/bin/env python
"""Class for pose storage."""
from __future__ import print_function
from tf.transformations import euler_from_quaternion
import numpy as np


class Pose(object):
    """Class to store Poses of the drone.

    The class is used to hold mainly poses but is also used
    to store other useful information needed to avoid using
    global variables.

    Args:
        x (float): x value of pose (optional)
        y (float): y value of pose (optional)
        z (float): z value of pose (optional)
        yaw (float): yaw value of pose (optional)
    Artributes:
        x (float): x value of pose (optional)
        y (float): y value of pose (optional)
        z (float): z value of pose (optional)
        yaw (float): yaw value of pose (optional)
        marker_ids (list): list of markers currently in frame
        current_marker_id (int): current id in use
        max_found (bool): true if max id of markers if found

    """

    def __init__(self, x=0, y=0, z=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.marker_ids = list()
        self.current_marker_id = None
        self.max_found = False

    def __str__(self):
        return "x: {}    y: {}   z: {}   yaw: {}".format(
            self.x, self.y, self.z, self.yaw)

    def convert_geometry_transform_to_pose(
            self, transform, remap=['x', 'y', 'z', 1]):
        """Convert geometry_msgs.msg.TransformStamped and stores to pose.

        Args:
            transform (geometry_msgs.msg.TransformStamped/Pose): transform to be stored.
            remap (list): any change of axis needed and which angle to use for yaw.

        Example:
            use of remapping: convert x axis of tranform to be z of pose
            and use eular angle at index 2
            p.convert_geometry_transform_to_pose(transform, remap=['z', 'y', 'x', 2])
        """
        try:
            euler = euler_from_quaternion((transform.orientation.x,
                                           transform.orientation.y,
                                           transform.orientation.z,
                                           transform.orientation.w
                                           ))
            # getting the attribute based on arg
            self.x = getattr(transform.position, remap[0])
            self.y = getattr(transform.position, remap[1])
            self.z = getattr(transform.position, remap[2])
            self.yaw = euler[remap[3]]
        # for some the attributes are different.
        # this may not be the best method but it works.
        except AttributeError:
            euler = euler_from_quaternion((transform.rotation.x,
                                           transform.rotation.y,
                                           transform.rotation.z,
                                           transform.rotation.w
                                           ))
            # getting the attribute based on arg
            self.x = getattr(transform.translation, remap[0])
            self.y = getattr(transform.translation, remap[1])
            self.z = getattr(transform.translation, remap[2])
            self.yaw = euler[remap[3]]

    def as_waypoints(self):
        """Return numpy array with x,y,z,yaw.

        Returns:
            numpy.array: array with x, y, z, yaw attributes.
        """
        return np.around(
            np.array([self.x, self.y, self.z, self.yaw]), decimals=3)

    def just_xy(self):
        """Return numpy array with just x and y values.

        These values are w.r.t the aruco plane

        Returns:
            numpy.array: array with y, z attributes.
        """
        return np.around(np.array([self.y, self.z]), decimals=3)

    def store_marker_ids(self, marker_ids):
        """Store marker ids.

        Args:
            marker_ids (list): list of markers currently detected.
        """
        self.marker_ids = marker_ids

    def get_marker_ids(self):
        """Return marker_ids.

        Returns:
            list: list of currently detected marker ids
        """
        return self.marker_ids

    def store_current_marker_id(self, current_marker_id):
        """Store current marker id.

        Args:
            current_marker_id (int): marker id currently in use.
        """
        self.current_marker_id = current_marker_id

    def get_current_marker_id(self):
        """Return current_marker_id.

        Returns:
            int: current marker id in use.
        """
        return self.current_marker_id

    def get_max_found(self):
        """Return max_found.

        Returns:
            bool: max_found, true if max id aruco found.
        """
        return self.max_found

    def set_max_found(self, max_found):
        """Store max_found.

        Args:
            max_found (bool): stores the value of max_found.
        """
        self.max_found = max_found
