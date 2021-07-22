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

transport = moteus_pi3hat.Pi3HatRouter(
    servo_bus_map = {
        1:[1], # KNEE
        2:[2], # HIP
    },
)

servos = {
    servo_id : moteus.Controller(id=servo_id, transport=transport)
    for servo_id in [1, 2]
}




async def main():

    # clearing any faults
    await transport.cycle([x.make_stop() for x in servos.values()])




if __name__ == "__main__":
    asyncio.run(main())

