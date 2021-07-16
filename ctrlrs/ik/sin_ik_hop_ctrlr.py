#!/usr/bin/env python

# sinusoidal hopping controller based on
# inverse kinematics set points controlled
# by a sine wave

#
# TODO: - Get accurate-ish leg lengths and joint angle limits
#       - Map encoder ticks to joint angles (radians, maybe make helper fxns for degrees-to-radian conversions)
#       - Implement try catches for out of bounds joint angles
#       - Implement behavior for sinusoidal y goal positions over time
#

import matplotlib.pyplot as plt
import numpy as np
from random import random


class sinIkHopCtrlr():

    def __init__(self):
        self.Kp = 12.5
        self.dt = 0.05
        self.l0 = 1
        self.l1 = 1.2
        self.q = np.array([[0],
                           [1]])
        self.show_animation = True

        if show_animation:
            plt.ion()

        def two_link_leg_ik(self, GOAL_TH):

        def sinusoidal_mv(self):
            theta0 = theta1 = 0.0
            while True:
                self.q[0] = 2.0 * random() - 1.0
                self.q[1] = 2.0 * random() - 1.0
                theta0, theta1 = two_link_leg_ik()
