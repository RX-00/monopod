#!/usr/bin/env python

# Inverse Kinematics of a two-link leg constrained to the real leg's limits
# KNEE: min: -0.65  max: -0.15
# HIP:  min: -0.51  max: +0.02
# ^^^ ROTATIONAL
#
# USAGE: Left-click the plot to set the
#      goal (x,y) position of the end
#      effector (foot)

#
# TODO: - Get accurate-ish leg lengths and joint angle limits
#       - Map encoder ticks to joint angles (radians, maybe make helper fxns for degrees-to-radian conversions)
#       - Implement try catches for out of bounds joint angles
#       - Implement behavior for sinusoidal y goal positions over time
#

import matplotlib.pyplot as plt
import numpy as np
from random import random
import time



class sinIkHopCtrlr():

    def __init__(self, Kp=25.0, dt=0.015, l0=0.1, l1=0.15, anim=True):
        self.Kp = Kp
        self.dt = dt
        self.l0 = l0
        self.l1 = l1
        # state vector for the foot point
        self.q = np.array([[0.1],  # x
                           [0.1]]) # y
        self.test_angle_rad = 0
        self.test_angle_deg = 0
        self.test_angle_enc = 0
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

            plt.xlim(-0.35, 0.35)
            plt.ylim(-0.35, 0.35)

            plt.show()
            plt.pause(self.dt / 1000)

        return foot


    def ang_diff(self, theta0, theta1):
        # Return difference b/w two angles in range -pi to +pi
        return (theta0 - theta1 + np.pi) % (2 * np.pi) - np.pi


    def lin_conv(self, old_val, old_min, old_max, new_min, new_max):
        old_rng = old_max - old_min
        if (old_rng == 0):
            new_val = new_min
        else:
            new_rng = (new_max - new_min)
            new_val = (((old_val - old_min) * new_rng) / old_rng) + new_min
        return new_val


    def sinusoidal_mv_anim(self):
        theta_hp = theta_kn = 0.0
        while True:
            now = time.time()
            self.q[0] = 0.0
            #self.q[1] = -0.25
            self.q[1] = 0.1 * np.sin(now) - 0.22
            theta_hp, theta_kn = self.two_link_leg_ik(
                des_eps=0.1, theta0=theta_hp, theta1=theta_kn)
            # print out the motor pos equiv:
            hp_pos = self.convert_rad_enc_hp(theta_hp)
            kn_pos = self.convert_rad_enc_kn(theta_kn)
            #print("time: ", now)
            print("hp: ", hp_pos)
            print("kn: ", kn_pos)


    def sinusoidal_mv(self, q_0, q_1):
        theta0 = theta1 = 0.0
        self.q[0] = q_0
        self.q[0] = q_1
        theta, theta1 = self.two_link_leg_ik(
            des_eps=0.1, theta0=theta0, theta1=theta1)
        return theta0, theta1


    def click(self, event):  # pragma: no cover
        q[0] = event.xdata
        q[1] = event.ydata


    def main_interactive(self):
        fig = plt.figure()
        fig.canvas.mpl_connect("button press event", self.click)
        # for trying to stop the animation w/ the esc key
        fig.canvas.mpl_connect("key_release_event", lambda event: [
            exit(0) if event.key == 'escape' else None
        ])
        self.two_link_leg_ik()


    # NOTE: MAKE SURE THE RANGE OF RADIANS IS APPROX TO IRL RANGE OF KNEE JOINT
    def convert_rad_enc_kn(self, theta):
        # linear conversion from theta angle to the knee motor limits
        goal_pos = self.lin_conv(theta, -np.pi/2.0, np.pi/2.0, -0.18, 0.18)
        return goal_pos

    def convert_rad_enc_hp(self, theta):
        # linear conversion from theta angle to the knee motor limits
        goal_pos = self.lin_conv(theta, -np.pi, np.pi, -0.48, 0.4)
        return goal_pos

    # NOTE: MAKE SURE THE RANGE OF RADIANS IS APPROX TO IRL RANGE OF KNEE JOINT
    def convert_enc_rad_kn(self, pos):
        # linear conversion from knee motor limits to theta angle
        theta = self.lin_conv(pos, -0.18, 0.18, -np.pi/2.0, np.pi/2.0)
        return theta

    def convert_enc_rad_hp(self, pos):
        # linear conversion from knee motor limits to theta angle
        theta = self.lin_conv(pos, -0.48, 0.4, -np.pi, np.pi)
        return theta

    def fwrd_kinematics(self):
        x = self.l0 * np.cos(self.theta0) + self.l1 * np.cos(self.theta0 + self.theta1)
        y = self.l0 * np.sin(self.theta0) + self.l1 * np.sin(self.theta0 + self.theta1)
        return x, y




if __name__ == "__main__":
    ctrlr = sinIkHopCtrlr()
    ctrlr.sinusoidal_mv_anim()
