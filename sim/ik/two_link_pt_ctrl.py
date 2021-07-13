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

# simulation params
Kp = 15  # TODO: why do they need a proportional gain here?
dt = 0.01

# link lengths
l1 = 1
l2 = 1.2

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
def two_link_leg_ik(goal_eps=0.0, theta1=0.0, theta2=0.0):
    # NOTE: double check if goal epsilon is needed (outside of animation)
    # TODO: manipulate goal_eps (change val, delete, etc.)
    x_prev, y_prev = None, None

    while True:
        try:
            if x is not None and y is not None:
                x_prev = x
                y_prev = y
        except ValueError as e:
            print("Unreachable Goal" + e)



if __name__ == "__main__":
    print(x)
    q[0] = 1
    print(x)
    x = 99     # x is redefined here, thus no longer tied to state vector q
    print(q[0])

    print("\n---------------------------\n")
    a = 0
    b = 0
    p = np.array([[a],
                  [b]])
    print(p)
    a = 99
    print(p)   # p is not updated dynamically by the value of a

