#!/usr/bin/python3 -B

# test script to individually test the knee motor

import asyncio
import math
import moteus
import moteus_pi3hat
import time
import argparse
import sys

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
commands = []

async def main_bare():
    # clearing any faults
    await transport.cycle([x.make_stop() for x in servos.values()])

    while True:
        now = time.time()
        # goal positions
        wave = np.sin(now)
        if (wave > 0):
            # crouch
            knee_pos = -0.175
            hip_pos = -0.065
        elif (wave < 0):
            # extend
            knee_pos = 0.387
            hip_pos = -0.244

        # irl position commands
        commands = [
            servos[1].make_position( # KNEE
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 0.5,
                stop_position = math.nan,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
            servos[2].make_position( # HIP
                position = hip_pos,
                velocity = 2.0,
                maximum_torque = 0.5,
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


if __name__ == "__main__":
    asyncio.run(main_bare())

