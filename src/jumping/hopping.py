#!/usr/bin/python3 -B

# Sinusoidal jumping program for the 2D monopod setup

from ctrlrs.ik.sin_ik_hop_ctrlr import sinIkHopCtrlr
from moteus_ctrlr.src.two_d_leg_class import Leg

import numpy as np
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
    # l0 = ~100 mm
    # l1 = ~150 mm
    # NOTE: sin controller: Kp,  dt,   l0,  l1,  animation)
    ctrlr = sinIkHopCtrlr(25.0, 0.015, 0.1, 0.15, False)
    ctrlr_x = ctrlr_y = 0

    kn_id = 1
    hp_id = 2
    theta_hp = theta_kn = 0
    wave = 1

    # create the leg class
    monopod = Leg(kn_id, hp_id) # NOTE: knee = 1, hip = 2

    # clearing any faults
    await monopod.stop_all_motors()

    # moving a bit
    while True:

        # NOTE TODO: if these desired positions don't work out
        #            make it so you create a square wave instead
        now = time.time()
        #wave = np.sin(now / 1.0)

        # desired pos sq sine wave:
        if (wave & 2 > 0): # crouch
            print("crouching")
            ctrlr.q[0] = 0.044
            ctrlr.q[1] =-0.164
        else:              # extend
            print("extending")
            ctrlr.q[0] =-0.039
            ctrlr.q[1] = 0.041

        # desired pos sine wave:
        #ctrlr.q[0] = 0.0
        #ctrlr.q[1] = 0.1 * np.sin(now) - 0.22


        theta_hp, theta_kn = ctrlr.two_link_leg_ik(
            des_eps=0.1, theta0=theta_hp, theta1=theta_kn)
        hp_pos = ctrlr.convert_rad_enc_hp(theta_hp)
        kn_pos = ctrlr.convert_rad_enc_kn(theta_kn)
        ctrlr_x, ctrlr_y = ctrlr.fwrd_kinematics(theta_hp, theta_kn)

        '''print("sim x: ", ctrlr.q[0])
        print("irl x: ", ctrlr_x)
        print("sim y: ", ctrlr.q[1])
        print("irl y: ", ctrlr_y)'''

        print("cmd pos: ", kn_pos)

        await monopod.set_motor_kn_cmds(kn_pos, # pos
                                        math.nan, # vel
                                        0.5,      # max_torque
                                        math.nan, # stop_pos
                                        -0.01,    # ffwd_torque
                                        math.nan, # watchdog_timeout
                                        True)     # query
        await monopod.set_motor_hp_cmds(hp_pos,
                                        math.nan,
                                        0.5,
                                        math.nan,
                                        -0.01,
                                        math.nan,
                                        True)

        results = await monopod.send_motor_cmds()

        # update the position readings to feedback into the ik ctrlr
        for result in results:
            if result.id == hp_id:
                theta_hp = ctrlr.convert_enc_rad_hp(moteus.Register.POSITION)
            elif result.id == kn_id:
                theta_kn = ctrlr.convert_enc_rad_kn(moteus.Register.POSITION)

        wave = wave + 1
        #print("now to wait for 5 seconds")
        await asyncio.sleep(5)



if __name__ == "__main__":
    asyncio.run(main())

