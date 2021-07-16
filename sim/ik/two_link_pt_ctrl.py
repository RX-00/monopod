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

# simulation params
Kp = 15  # TODO: why do they need a proportional gain here?
dt = 0.001

# link lengths
l0 = 1
l1 = 1.2

# init goal pos for the end-effector pos
q = np.array([[2],
              [0]])
x = q[0]
y = q[1]

show_animation = True

if show_animation:
    plt.ion() # turn on interactive mode


# Computes the ik for a planar 2DOF leg
# When out of bounds, rewrite q[0] and q[1]
# w/ previous valid values
def two_link_leg_ik(goal_eps=0.0, theta0=0.0, theta1=0.0):
    # NOTE: theta1_goal must be calc before theta0_goal
    global x, y
    x_prev, y_prev = None, None

    while True:
        try:
            if x is not None and y is not None:
                x_prev = x
                y_prev = y

            # NOTE: assuming the base point is the origin (0,0) for the dist formula
            if np.sqrt(x**2 + y**2) > (l0 + l1):
                theta1_goal = 0
                # if the dist from base pt to end eff is greater than links then
                # the solution is just to go to a straight line to the goal pt

            else:
                theta1_goal = np.arccos(
                    (x**2 + y**2 - l0**2 - l1**2) / (2 * l0 * l1))

            beta_angle = np.math.atan2(l1 * np.sin(theta1_goal),
                                       (l0 + l1 * np.cos(theta1_goal)))
            gamma_angle = np.math.atan2(y, x)
            theta0_goal = gamma_angle - beta_angle

            if theta0_goal < 0:
                theta1_goal = -theta1_goal
                beta_angle = np.math.atan2(l1 * np.sin(theta1_goal),
                                            (l0 + l1 * np.cos(theta1_goal)))
                theta0_goal = gamma_angle - np.math.atan2(y, x)


            # apply proportional gain and time step to output angles
            theta0 = theta0 + Kp * ang_diff(theta0_goal, theta0) * dt
            theta1 = theta1 + Kp * ang_diff(theta1_goal, theta1) * dt


        except ValueError as e:
            print("Unreachable Goal" + e)

        # go back to the last valid point
        except TypeError:
            x = x_prev
            y = y_prev

        ft = plot_leg(theta0, theta1, x, y)

        # check the goal
        dist_2_goal = None
        if x is not None and y is not None:
            dist_2_goal = np.hypot(ft[0] - x, ft[1] - y)

        if abs(dist_2_goal) < goal_eps and x is not None:
            return theta0, theta1


def plot_leg(theta1, theta2, target_x, target_y):  # pragma: no cover
    shoulder = np.array([0, 0])
    elbow = shoulder + np.array([l0 * np.cos(theta1), l0 * np.sin(theta1)])
    wrist = elbow + \
        np.array([l1 * np.cos(theta1 + theta2), l1 * np.sin(theta1 + theta2)])

    if show_animation:
        plt.cla()

        plt.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], 'k-')
        plt.plot([elbow[0], wrist[0]], [elbow[1], wrist[1]], 'k-')

        plt.plot(shoulder[0], shoulder[1], 'ro')
        plt.plot(elbow[0], elbow[1], 'ro')
        plt.plot(wrist[0], wrist[1], 'ro')

        plt.plot([wrist[0], target_x], [wrist[1], target_y], 'g--')
        plt.plot(target_x, target_y, 'g*')

        plt.xlim(-2, 2)
        plt.ylim(-2, 2)

        plt.show()
        plt.pause(dt)

    return wrist


def ang_diff(theta0, theta1):
    # Return difference b/w two angles in range -pi to +pi
    return (theta0 - theta1 + np.pi) % (2 * np.pi) - np.pi


def click(event):
    global x, y
    x = event.xdata
    y = event.ydata


def rand_goal_animation():
    global x, y
    theta0 = theta1 = 0.0
    for i in range(5):
        x = 2.0 * random() -1.0
        y = 2.0 * random() -1.0
        theta0, theta1 = two_link_leg_ik(
            goal_eps=0, theta0=theta0, theta1=theta1)


def main(): # pragma: no cover
    fig = plt.figure()
    fig.canvas.mpl_connect("button_press_event", click)
    # for stopping simulation with the esc key.
    fig.canvas.mpl_connect('key_release_event', lambda event: [
                           exit(0) if event.key == 'escape' else None])
    two_link_leg_ik()



if __name__ == "__main__":
    main()
