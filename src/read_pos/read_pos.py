#!/usr/bin/python3 -B

# Sinusoidal jumping program for the 2D monopod setup

from moteus_ctrlr.src.two_d_leg_class import Leg

import asyncio
import math
import moteus
import time
import argparse
import sys


async def main():
    # create the leg class
    monopod = Leg(1, 2)

    # clearing any faults
    await monopod.stop_all_motors()

    now = time.time()
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

    # moving a bit
    while True:
        await monopod.stop_all_motors()

        '''
        print(", ".join(
            f"({result.id} " +
            f"{result.values[moteus.Register.POSITION]} " +
            f"{result.values[moteus.Register.VELOCITY]})"
            for result in results))
        '''

        print(", ".join(
            f"({result.id} " +
            f"{result.values[moteus.Register.POSITION]} "
            for result in results))



        #await asyncio.sleep(1)



if __name__ == "__main__":
    asyncio.run(main())

