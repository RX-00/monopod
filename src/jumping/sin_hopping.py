#!/usr/bin/python3 -B

# Sinusoidal jumping program for the 2D monopod setup

from ctrlrs.ik.sin_ik_hop_ctrlr import sinIkHopCtrlr
from moteus_ctrlr.src.two_d_leg_class import Leg

import asyncio
import math
import moteus
import time
import argparse
import sys



# TODO: check out and if needed tune/change the motor's internal PID
#       in order to increase damping!
# TODO: setup the links and coordinates to be in meters


async def main():
    # create the controller class
    # NOTE: sin controller: Kp,  dt,   l0,  l1,  animation)
    ctrlr = sinIkHopCtrlr(25.0, 0.015, 1.0, 1.2, False)

    knee = 1
    hip = 2
    theta_hp = theta_kn = 0
    wave = 0

    # create the leg class
    monopod = Leg(knee, hip) # NOTE: knee = 1, hip = 2

    # clearing any faults
    await monopod.stop_all_motors()

    # moving a bit
    while True:
        now = time.time()

        #if (wave > 0):
        #    ctrlr.q[0] = 0.0
        #    ctrlr.q[1] = 0.0
        #elif (wave < 0):
        #    ctrlr.q[0] = 0.0
        #    ctrlr.q[1] = 0.0


        # desired pos:
        ctrlr.q[0] = 0.0
        ctrlr.q[1] = 1.0 * np.sin(now) - 2.0
        # NOTE TODO: if these desired positions don't work out
        #            make it so you create a square wave instead
        #wave = np.sin(now / 1.0)

        theta_hp, theta_kn = ctrlr.two_link_leg_ik(
            des_eps=0.1, theta0=theta_hp, theta1=theta_kn)
        hp_pos = ctrlr.convert_rad_enc_hp(theta_hp)
        kn_pos = ctrlr.convert_rad_enc_kn(theta_kn)

        await monopod.set_motor_kn_cmds(kn_pos, # pos
                                        math.nan, # vel
                                        2.0,      # max_torque
                                        math.nan, # stop_pos
                                        -0.01,    # ffwd_torque
                                        math.nan, # watchdog_timeout
                                        True)     # query
        await monopod.set_motor_hp_cmds(hp_pos,
                                        math.nan,
                                        2.0,
                                        math.nan,
                                        -0.01,
                                        math.nan,
                                        True)

        results = await monopod.send_motor_cmds()

        # update the position readings to feedback into the ik ctrlr
        for result in results:
            if result.id == hip:
                theta_hp = ctrlr.convert_pos_rad_hp(moteus.Register.POSITION)
            elif result.id == knee:
                theta_kn = ctrlr.convert_pos_rad_kn(moteus.Register.POSITION)

        await asyncio.sleep(0.1)



if __name__ == "__main__":
    asyncio.run(main())

