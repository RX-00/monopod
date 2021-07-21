#!/usr/bin/env python

# sinusoidal hopping controller based on
# analytical inverse kinematics set points controlled
# by a sine wave with a P control loop

#
# TODO: - Perhaps implement Ki and Kd gains for more full robust PID control
#

import matplotlib.pyplot as plt
import numpy as np
from random import random
import sys
import time


class sinIkHopCtrlr():

    def __init__(self, Kp=25.0, dt=0.015, l0=1.0, l1=1.2, anim=True):
        self.Kp = Kp
        self.dt = dt
        self.l0 = l0
        self.l1 = l1
        # state vector for the foot point
        self.q = np.array([[1],  # x
                           [1]]) # y
        self.show_animation = anim
        if self.show_animation:
            plt.ion()


    # Computes the ik for a planar 2DOF leg
    # When out of bounds, rewrite q[0] and q[1]
    # w/ previous valid values
    def two_link_leg_ik(self, des_eps=0.0, theta0=0.0, theta1=0.0):
        x_prev, y_prev = None, None

        while True:
            try:
                if float(self.q[0]) is not None and float(self.q[1]) is not None:
                    x_prev = float(self.q[0])
                    y_prev = float(self.q[1])

                # NOTE: assuming the base point is the origin (0,0) for the dist formula
                if np.sqrt(float(self.q[0])**2 + float(self.q[1])**2) > (self.l0 + self.l1):
                    theta1_des = 0

                else:
                    theta1_des = np.arccos(
                        (float(self.q[0])**2 + float(self.q[1])**2 - self.l0**2 - self.l1**2) /
                        (2 * self.l0 * self.l1))

                beta_angle = np.math.atan2(self.l1 * np.sin(theta1_des),\
                                           (self.l0 + self.l1 * np.cos(theta1_des)))
                gamma_angle = np.math.atan2(float(self.q[1]), float(self.q[0]))
                theta0_des = gamma_angle - beta_angle

                if theta0_des < 0:
                    theta1_des = -theta1_des
                    beta_angle = np.math.atan2(self.l1 * np.sin(theta1_des),
                                               (self.l0 + self.l1 * np.cos(theta1_des)))
                    theta0_des = gamma_angle - beta_angle

                # apply proportional gain and time step to output angles
                theta0 = theta0 + self.Kp * self.ang_diff(theta0_des, theta0) * self.dt
                theta1 = theta1 + self.Kp * self.ang_diff(theta1_des, theta1) * self.dt

            except ValueError as e:
                print("Unreachable des"+e)

            # go back to the last valid point
            except TypeError:
                x = x_prev
                y = y_prev

            foot = self.plot_leg(theta0, theta1, float(self.q[0]), float(self.q[1]))

                # check desired point
            dist_to_des = None
            if float(self.q[0]) is not None and float(self.q[1]) is not None:
                dist_to_des = np.hypot(foot[0] - float(self.q[0]), foot[1] - float(self.q[1]))

            if abs(dist_to_des) < des_eps and float(self.q[0]) is not None:
                return theta0, theta1


    def plot_leg(self, theta0, theta1, target_x, target_y):  # pragma: no cover
        hip = np.array([0, 0])
        knee = hip + np.array([self.l0 * np.cos(theta0), self.l0 * np.sin(theta0)])
        foot = knee + \
            np.array([self.l1 * np.cos(theta0 + theta1), self.l1 * np.sin(theta0 + theta1)])

        if self.show_animation:
            plt.cla()

            plt.plot([hip[0], knee[0]], [hip[1], knee[1]], 'k-')
            plt.plot([knee[0], foot[0]], [knee[1], foot[1]], 'k-')

            plt.plot(hip[0], hip[1], 'ro')
            plt.plot(knee[0], knee[1], 'ro')
            plt.plot(foot[0], foot[1], 'ro')

            plt.plot([foot[0], target_x], [foot[1], target_y], 'g--')
            plt.plot(target_x, target_y, 'g*')

            plt.xlim(-2, 2)
            plt.ylim(-2, 2)

            plt.show()
            plt.pause(self.dt / 1000)

        return foot


    def ang_diff(self, theta0, theta1):
        # Return difference b/w two angles in range -pi to +pi
        return (theta0 - theta1 + np.pi) % (2 * np.pi) - np.pi


    def sinusoidal_mv_anim(self):
        theta0 = theta1 = 0.0
        while True:
            now = time.time()
            self.q[0] = 0.0
            self.q[1] = 1.0 * np.sin(now) - 2.0
            theta0, theta1 = self.two_link_leg_ik(
                des_eps=0.1, theta0=theta0, theta1=theta1)
            # print out the motor pos equiv:
            hp_pos = self.convert_rad_enc_hp(theta0)
            kn_pos = self.convert_rad_enc_kn(theta1)
            print("time: ", now)
            print("hp: ", hp_pos)
            print("kn: ", kn_pos)

    def sinusoidal_mv(self, q_0, q_1):
        theta0 = theta1 = 0.0
        self.q[0] = q_0
        self.q[0] = q_1
        theta, theta1 = self.two_link_leg_ik(
            des_eps=0.1, theta0=theta0, theta1=theta1)
        return theta0, theta1

    def convert_rad_enc_kn(self, theta):
        # clamp the theta angle b/w the knee motor limits
        goal_pos = np.interp(theta,[-np.pi, np.pi], [-0.15, -0.65])
        return goal_pos

    def convert_rad_enc_hp(self, theta):
        # clamp the theta angle b/w the hip motor limits
        goal_pos = np.interp(theta,[-np.pi, np.pi], [-0.51, 0.02])
        return goal_pos

    def convert_pos_rad_kn(self, pos):
        theta = np.interp(pos, [-0.15, -0.65], [-np.pi, np.pi])
        return theta

    def convert_pos_rad_hp(self, pos):
        theta = np.interp(pos, [-0.51, 0.02], [-np.pi, np.pi])
        return theta



if __name__ == "__main__":
    ctrlr = sinIkHopCtrlr()
    ctrlr.sinusoidal_mv_anim()
