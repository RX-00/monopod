#!/usr/bin/env python

# Inverse Kinematics of a two-link leg
# USE: Left-click the plot to set the
#      goal (x,y) position of the end
#      effector (foot)

import matplotlib.pyplot as plt
import numpy as np

# simulation params
Kp = 15  # TODO: why do they need a proportional gain here?
dt = 0.01

# link lengths
l1 = 1
l2 = 1.2



