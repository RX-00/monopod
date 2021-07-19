#!/usr/bin/python3 -B

# Sinusoidal jumping program for the 2D monopod setup

from ctrlrs.ik.sin_ik_hop_ctrlr import sinIkHopCtrlr
from moteus.src.two_d_leg_class import Leg

import asyncio
import math
import moteus
import time
import argparse
import sys


async def main():
    # create the controller class
    # NOTE: sin controller: Kp,  dt,   l0,  l1,  animation)
    ctrlr = sinIkHopCtrlr(25.0, 0.015, 1.0, 1.2, False)

    # create the leg class
    monopod = Leg(1, 2)

    # clearing any faults
    await monopod.stop_all_motors()

    # moving a bit
    while True:
        now = time.tim()
        vel = 0.2 * math.sin(now)
        vel1 = 0.2 * math.sin(now + 1)

        await monopod.set_motor_kn_cmds(math.nan, # pos
                                        vel,      # vel
                                        2.0,      # max_torque
                                        math.nan, # stop_pos
                                        -0.01,    # ffwd_torque
                                        math.nan, # watchdog_timeout
                                        True)     # query
        await monopod.set_motor_hp_cmds(math.nan,
                                        vel1,
                                        2.0,
                                        math.nan,
                                        -0.01,
                                        math.nan,
                                        True)

        results = await monopod.send_motor_cmds()

        await asyncio.sleep(1)



if __name__ == "__main__":
    asyncio.run(main())

