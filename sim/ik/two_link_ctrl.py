#!/usr/bin/env python

# Inverse Kinematics of a two-link leg
# USE: Left-click the plot to set the
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


# Similation parameters
Kp = 12.5
dt = 0.05

# Link lengths
l0 = 1
l1 = 1.2

# Set initial goal position to the initial end-effector position
x = 2
y = 0

show_animation = True

if show_animation:
    plt.ion()


# Computes the ik for a planar 2DOF leg
# When out of bounds, rewrite q[0] and q[1]
# w/ previous valid values
def two_link_leg_ik(GOAL_TH=0.0, theta1=0.0, theta2=0.0):
    # NOTE: theta2_goal must be calc before theta1_goal
    global x, y
    x_prev, y_prev = None, None
    while True:
        try:
            if x is not None and y is not None:
                x_prev = x
                y_prev = y

            # NOTE: assuming the base point is the origin (0,0) for the dist formula
            if np.sqrt(x**2 + y**2) > (l0 + l1):
                theta2_goal = 0

            else:
                theta2_goal = np.arccos(
                    (x**2 + y**2 - l0**2 - l1**2) / (2 * l0 * l1))

            beta_angle = np.math.atan2(l1 * np.sin(theta2_goal),
                                (l0 + l1 * np.cos(theta2_goal)))
            gamma_angle = np.math.atan2(y, x)
            theta1_goal = gamma_angle - beta_angle

            if theta1_goal < 0:
                theta2_goal = -theta2_goal
                beta_angle = np.math.atan2(l1 * np.sin(theta2_goal),
                                    (l0 + l1 * np.cos(theta2_goal)))
                theta1_goal = gamma_angle - beta_angle

            # apply proportional gain and time step to output angles
            theta1 = theta1 + Kp * ang_diff(theta1_goal, theta1) * dt
            theta2 = theta2 + Kp * ang_diff(theta2_goal, theta2) * dt
        except ValueError as e:
            print("Unreachable goal"+e)

        # go back to the last valid point
        except TypeError:
            x = x_prev
            y = y_prev

        foot = plot_leg(theta1, theta2, x, y)

        # check goal
        d2goal = None
        if x is not None and y is not None:
            d2goal = np.hypot(foot[0] - x, foot[1] - y)

        if abs(d2goal) < GOAL_TH and x is not None:
            return theta1, theta2


def plot_leg(theta1, theta2, target_x, target_y):  # pragma: no cover
    hip = np.array([0, 0])
    knee = hip + np.array([l0 * np.cos(theta1), l0 * np.sin(theta1)])
    foot = knee + \
        np.array([l1 * np.cos(theta1 + theta2), l1 * np.sin(theta1 + theta2)])

    if show_animation:
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
        plt.pause(dt)

    return foot


def ang_diff(theta0, theta1):
    # Return difference b/w two angles in range -pi to +pi
    return (theta0 - theta1 + np.pi) % (2 * np.pi) - np.pi


def click(event):  # pragma: no cover
    global x, y
    x = event.xdata
    y = event.ydata


def rand_goal_animation():
    global x, y
    theta1 = theta2 = 0.0
    for i in range(5):
        x = 2.0 * random() - 1.0
        y = 2.0 * random() - 1.0
        theta1, theta2 = two_link_leg_ik(
            GOAL_TH=0.01, theta1=theta1, theta2=theta2)


def main():  # pragma: no cover
    fig = plt.figure()
    fig.canvas.mpl_connect("button_press_event", click)
    # for stopping simulation with the esc key.
    fig.canvas.mpl_connect('key_release_event', lambda event: [
                           exit(0) if event.key == 'escape' else None])
    two_link_leg_ik()


if __name__ == "__main__":
    main()
