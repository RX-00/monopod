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

    '''
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
    '''

    # Reading each servo's position through each moteus controller corresponding
    # to each servo_id in Leg.servos
    while True:
        # update the current pos to the closest one which is consistent with an output pos
        await monopod.servos[monopod.hip_pitch].set_rezero(0.0, query=True)
        await monopod.servos[monopod.knee].set_rezero(0.0, query=True)

        result_kn = result_hp = None
        while (result_hp and result_kn) is None:
            result_hp = await monopod.servos[monopod.hip_pitch].query()
            result_kn = await monopod.servos[monopod.knee].query()

        print("hip pos: ", result_hp.values[moteus.Register.POSITION])
        print("knee pos: ", result_kn.values[moteus.Register.POSITION])
        # NOTE: if stop_all_motors dooesn't work then try this:
        '''
        await monopod.servos[monopod.hip_pitch].set_stop()
        await monopod.servos[monopod.knee].set_stop()
        '''
        await monopod.stop_all_motors()




if __name__ == "__main__":
    asyncio.run(main())

