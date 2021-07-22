#!/usr/bin/python3 -B

# Programming for passively reading the joint angles on the monopod

from moteus_ctrlr.src.two_d_leg_class import Leg
from ctrlrs.ik.sin_ik_hop_ctrlr import sinIkHopCtrlr

import asyncio
import math
import moteus
import time
import argparse
import sys


async def main():
    kn_id = 1
    hp_id = 2
    ctrlr_x = ctrlr_y = 0

    # create the leg class
    monopod = Leg(kn_id, hp_id) # TODO: double check the motor port id's

    # create controller class
    # l0 = ~100 mm
    # l1 = ~150 mm
    # NOTE: sin controller: Kp,  dt,   l0,  l1,  animation)
    ctrlr = sinIkHopCtrlr(25.0, 0.015, 0.1, 0.15, False)

    # clearing any faults
    await monopod.stop_all_motors()

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

        # now we have all the info about the actual monopod's joint positions,
        # so now we must convert & feedback that info to the ctrlr
        ctrlr.theta0 = ctrlr.convert_enc_rad_hp(result_hp.values[moteus.Register.POSITION])
        ctrlr.theta1 = ctrlr.convert_enc_rad_kn(result_kn.values[moteus.Register.POSITION])
        ctrlr_x, ctrlr_y = ctrlr.fwrd_kinematics()


        print("hip pos: ", result_hp.values[moteus.Register.POSITION])
        print("knee pos: ", result_kn.values[moteus.Register.POSITION])

        #print("theta0: ", np.degrees(ctrlr.theta0.degrees()))
        #print("theta1: ", np.degrees(ctrlr.theta1.degrees()))
        #print("q[0]: ", ctrlr_x)
        #print("q[1]: ", ctrlr_y)
        #print("---------------------------------------------------------")

        await monopod.stop_all_motors()




if __name__ == "__main__":
    asyncio.run(main())

