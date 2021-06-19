#!/usr/bin/python3 -B

# Jumping program for the 2D monoped setup
# This program should be extendable to add more servos (at later stages)
# such as, roll and yaw servos in the hip and also two ankle servos
# NOTE: ankle servos may end up being hobby servos (but this will be designed more in the future)
#

import asyncio
import math
import moteus
import moteus_pi3hat
import time
import argparse
import sys


'''
TODO:

Make a class that defines each leg (extendable to 4 servos)

Figure out how to make the spring jumping up and down virtual leg
1. get the measurements for the leg links
2. figure out the lowest possible length of the virtual leg spring
3. then figure out the IK that needs to be mapped to the input positions of the servos

'''


'''
NOTE:
KNEE servopos.position_min = -0.65
KNEE servopos.position_max = -0.15

HIP servopos.position_min = -0.51
HIP servopos.position_max = +0.02
'''


MAX_POS_KN = -0.15 - 0.00
MIN_POS_KN = -0.65 + 0.00

MAX_POS_HP =  0.02 - 0.00
MIN_POS_HP = -0.51 + 0.00





class Leg:
    def __init__(self, id):
        self.

    async def do_foo(self):
        return False;



async def main():
    print("Starting jumping...")



if __name__ == '__main__':
    asyncio.run(main())
















