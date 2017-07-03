#!/usr/bin/env python
from __future__ import print_function

import rospy

from geometry_msgs import msg
from tf.transformations import quaternion_from_euler

import numpy as np


class pFilter(object):
    """Manipulator for only pose quantities
    
    Artibutes:
        state (numpy.array): the current state of quanity
        var (float): variance of quantity
        prev (float): previous value of quantity
    """
    def __init__(self):
        self.state = 0
        self.var = 0.001
        self.prev = None

    def observe(self, obs, obs_var):
        """Make an observation step
        
        Args:
            obs (numpy.array): observation array
            obs_var (float): variance array
        """
        w = self.var / (self.var + obs_var)
        self.state = (1 - w) * self.state + w * obs
        self.var = self.var * obs_var / (self.var + obs_var)

    def predict(self, dt, speed_var, control_gain):
        """Prediction step

        Args:
            dt (float): time difference
            speed_var (float): varience in speed
            control_gain (numpy.array): control values sent to drone
        """
        self.state += control_gain
        self.var += speed_var * dt * dt


class pvFilter(object):
    """Manipulator for pose and velocity quantities
    
    Artibutes:
        state (numpy.array): the current state of quanity
        var (numpy.array): covariance of quantity
        prev (numpy.array): previous value of quantity
    """
    def __init__(self):
        self.state = np.zeros(2)
        self.var = np.array([[0.001, 0.], [0., 0.001]])
        self.prev = None

    def observe_pose(self, obs, obs_var):
        """Make an observation step
        
        Args:
            obs (numpy.array): observation array
            obs_var (numpy.array): variance array
        """
        K = self.var[0] / (obs_var + self.var[0, 0])
        self.state += np.dot(K, (obs - self.state[0]))
        tmp = np.eye(2)
        tmp[0, 0] -= K[0]
        tmp[1, 0] -= K[1]
        self.var = np.dot(self.var, tmp)

    def observe_speed(self, obs, obs_var):
        """Make an observation step
        
        Args:
            obs (numpy.array): observation array
            obs_var (numpy.array): variance array
        """
        K = self.var[1] / (obs_var + self.var[1, 1])
        self.state += np.dot(K, (obs - self.state[1]))
        tmp = np.eye(2)
        tmp[0, 1] -= K[0]
        tmp[1, 1] -= K[1]
        self.var = np.dot(self.var, tmp)

    # calculates prediction variance matrix based on gaussian acceleration as
    # error.
    def predict_gaussion_accel(
            self,
            dt,
            acceleration_var,
            control_gains,
            cov_fac=1,
            speed_var_fac=1):
        """Prediction step using gaussion acceleration

        Args:
            dt (float): time difference
            acceleration_var (float): varience in acceleration
            control_gain (numpy.array): control values sent to drone
            cov_fac (float): covarience factor
            speed_var_fac (float): speed varience factor
        """
        G = np.eye(2)
        G[0, 1] = dt

        self.state = np.dot(G, self.state) + control_gains
        self.var = np.dot(np.dot(G, self.var), G.T)
        self.var[0, 0] += acceleration_var * 0.25 * dt * dt * dt * dt
        self.var[1, 0] += cov_fac * acceleration_var * 0.5 * dt * dt * dt * 4
        self.var[0, 1] += cov_fac * acceleration_var * 0.5 * dt * dt * dt * 4
        self.var[1, 1] += speed_var_fac * \
            acceleration_var * 1 * dt * dt * 4 * 4

    # calculates prediction using the given uncertainty matrix
    # vars is var(0) var(1) covar(0,1)
    def predict(self, dt, vars, control_gains):
        """Prediction step

        Args:
            dt (float): time difference
            vars (numpy.array): varience
            control_gains (numpy.array): control values sent to drone
        """
        G = np.eye(2)
        G[0, 1] = dt

        self.state = np.dot(G, self.state) + control_gains
        self.var = np.dot(np.dot(G, self.var), G.T)
        self.var[0, 0] += vars[0]
        self.var[1, 0] += vars[2]
        self.var[0, 1] += vars[2]
        self.var[1, 1] += vars[1]


class extendedKalmanFilter(object):
    """Class resposible to exceute EKF

    This is NOT fully developed and tested, may not behave
    as it is expected to.
    """
    def __init__(self):
        self.x = pvFilter()
        self.y = pvFilter()
        self.z = pvFilter()
        self.roll = pFilter()
        self.pitch = pFilter()
        self.yaw = pvFilter()
        # increased because prediction based on control command is damn
        # inaccurate.
        self.var_speed_error = 360 * 360 * 16
        self.var_speed_observation_xy = 2 * 2
        self.var_accel_error_yaw = 360 * 360
        self.var_accel_error_xy = 8 * 8
        self.var_pose_observation_rp_imu = 1 * 1
        self.var_pose_observation_z_IMU = 0.25 * 0.25
        self.var_pose_observation_xy = 0.2 * 0.2
        self.var_pose_observation_z_aruco = 0.08 * 0.08
        self.var_pose_observation_rp_aruco = 3 * 3
        self.var_pose_observation_yaw = 3 * 3

    def __str__(self):
        return "x: {}       y: {}       z: {}       yaw: {}".format(
            np.around(
                self.x.state[0], 3), np.around(
                self.y.state[0], 3), np.around(
                self.z.state[0], 3), np.around(
                    self.yaw.state[0], 3))

    def get_current_pose(self):
        """Gets current pose/state of the drone"""
        pose = msg.Pose()
        pose.position.x = self.x.state[0]
        pose.position.y = self.y.state[0]
        pose.position.z = self.z.state[0]
        quat = quaternion_from_euler(
            self.pitch.state,
            self.roll.state,
            self.yaw.state[0])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def prediction(self, active_control, dt):
        """Make a prediciton Step"""
        # proportionality constants
        c1 = 0.58
        c2 = 17.8
        c3 = 10
        c4 = 35
        c5 = 10
        c6 = 25
        c7 = 1.4
        c8 = 1.0

        if dt <= 0:
            return

        # predict roll, pitch, yaw
        roll_control_gain = dt * c3 * \
            (c4 * max(-0.5, min(0.5, active_control.linear.y)) - self.roll.state)
        pitch_control_gain = dt * c3 * \
            (c4 * max(-0.5, min(0.5, active_control.linear.x)) - self.pitch.state)
        yaw_speed_control_gain = dt * c5 * \
            (c6 * active_control.angular.z - self.yaw.state[1])

        yaw_rad = np.radians(self.yaw.state[0])
        roll_rad = np.radians(self.roll.state)
        pitch_rad = np.radians(self.pitch.state)

        forceX = np.cos(yaw_rad) * np.sin(roll_rad) * \
            np.cos(pitch_rad) - np.sin(yaw_rad) * np.sin(pitch_rad)
        forceY = - np.sin(yaw_rad) * np.sin(roll_rad) * \
            np.cos(pitch_rad) - np.cos(yaw_rad) * np.sin(pitch_rad)

        mul_fac = 1
        if active_control.linear.z < 0:
            mul_fac = 2

        vx_gain = dt * c1 * (c2 * forceX - self.x.state[1])
        vy_gain = dt * c1 * (c2 * forceY - self.y.state[1])
        vz_gain = dt * c7 * (c8 * active_control.linear.z *
                             mul_fac - self.z.state[1])

        lastVXGain = vx_gain
        lastVYGain = vy_gain
        lastPredictedRoll = self.roll.state
        lastPredictedPitch = self.pitch.state

        # see  if this is needed?
        # yaw.state[0] =  angleFromTo(yaw.state[0],-180,180)
        self.roll.predict(dt, self.var_speed_error, roll_control_gain)
        self.pitch.predict(dt, self.var_speed_error, pitch_control_gain)
        self.yaw.predict_gaussion_accel(dt, self.var_accel_error_yaw, np.array(
            [dt * yaw_speed_control_gain / 2, yaw_speed_control_gain]), 1, 5 * 5)
        # yaw.state[0] =  angleFromTo(yaw.state[0],-180,180);

        self.x.predict_gaussion_accel(dt, self.var_accel_error_xy, np.array([
                                      dt * vx_gain / 2, vx_gain]), 0.0001)
        self.y.predict_gaussion_accel(dt, self.var_accel_error_xy, np.array([
                                      dt * vy_gain / 2, vy_gain]), 0.0001)
        self.z.predict(dt, np.array([dt *
                                     dt *
                                     dt *
                                     dt, 9 *
                                     dt, dt *
                                     dt *
                                     dt *
                                     3]), np.array([dt *
                                                    vz_gain /
                                                    2, vz_gain]))
