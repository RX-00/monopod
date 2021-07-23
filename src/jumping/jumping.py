#!/usr/bin/python3 -B

# Sinusoidal jumping program for the 2D monopod setup

from ctrlrs.ik.sin_ik_hop_ctrlr import sinIkHopCtrlr
from moteus_ctrlr.src.two_d_leg_class import Leg

import numpy as np
import asyncio
import math
import moteus
import moteus_pi3hat
import time
import argparse
import sys



# TODO: check out and if needed tune/change the motor's internal PID
#       in order to increase damping!

knee = 1
hip  = 2

transport = moteus_pi3hat.Pi3HatRouter(
    servo_bus_map = {
        1:[knee], # KNEE
        2:[hip], # HIP
    },
)

servos = {
    servo_id : moteus.Controller(id=servo_id, transport=transport)
    for servo_id in [1, 2]
}

# NOTE: sin controller: Kp,  dt,   l0,  l1,  animation)
ctrlr = sinIkHopCtrlr(25.0, 0.015, 0.1, 0.15, False)
ctrlr_x = ctrlr_y = 0
theta0 = 0 # hip angle
theta1 = 0 # knee angle
commands = []
ctrlr.q[0] = 0 # x ft pos in sim
ctrlr.q[1] = 0 # y ft pos in sim


async def main():

    # clearing any faults
    await transport.cycle([x.make_stop() for x in servos.values()])

    # update the current pos to the closest one which is consistent with an output pos
    await servos[0].set_rezero(0.0, query=True)
    await servos[1].set_rezero(0.0, query=True)

    result_kn = result_hp = None
    while (result_hp and result_kn) is None:
        result_kn = await servos[0].query()
        result_hp = await servos[1].query()

    # now we have all the info about the actual monopod's joint positions,
    # so now we must convert & feedback that info to the ctrlr
    ctrlr.theta0 = ctrlr.convert_enc_rad_hp(result_hp.values[moteus.Register.POSITION])
    ctrlr.theta1 = ctrlr.convert_enc_rad_kn(result_kn.values[moteus.Register.POSITION])
    ctrlr_x, ctrlr_y = ctrlr.fwrd_kinematics()
    ctrlr.q[0] = ctrlr_x
    ctrlr.q[1] = ctrlr_y

    print("x: ", ctrlr.q[0])
    print("y: ", ctrlr.q[1])


    while True:
        #now = time.time()

        theta0, theta1 = ctrlr.two_link_leg_ik(
            des_eps=0.1, theta0=theta0, theta1=theta1)
        hp_pos = ctrlr.convert_rad_enc_hp(theta0)
        kn_pos = ctrlr.convert_rad_enc_kn(theta1)

        # position commands
        commands = [
            servos[1].make_position( # KNEE
                position = ctrlr.convert_rad_enc_kn(kn_pos)
                velocity = math.nan
                #position = math.nan,
                #velocity = 0.2 * math.sin(now),
                maximum_torque = 2.0,
                stop_position = math.nan,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
            servos[2].make_position( # HIP
                position = ctrlr.convert_rad_enc_hp(hp_pos)
                velocity = math.nan
                #position = math.nan,
                #velocity = 0.2 * math.sin(now + 1),
                maximum_torque = 2.0,
                stop_position = math.nan,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
        ]

        results = await transport.cycle(commands)

        # NOTE: it is possible to not receive responses from all servos
        #       for which a query was requested


        # We will wait 20ms between cycles. By default, each servo has
        # a watchdog timeout, where if no CAN command is received for
        # 100mc the controller will enter a latched fault state
        await asyncio.sleep(0.02)

        ctrlr.q[0]
        ctrlr.q[1]




if __name__ == "__main__":
    asyncio.run(main())

