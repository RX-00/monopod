#!/usr/bin/python3 -B

# general motor testing of moving hip (hp) & knee (kn) joints of simple
# monopod robot through the moteus controller and the moteus pi3hat
#
# This commands multiple servos connected to a pi3hat. It uses the cycle()
# method in order to optimally use the pi3hat bandwidth

import asyncio
import math
import moteus
import moteus_pi3hat
import time
import argparse
import sys


POS_OFFSET = 0.0005

# unwrapped pos with neg
MAX_POS_HP = 0.00  # same as 1.00
MIN_POS_HP = -0.5  # same as 0.50
MID_POS_HP = -0.25 # same as 0.75

MAX_POS_HP_SOFT_STOP = MAX_POS_HP + POS_OFFSET
MIN_POS_HP_SOFT_STOP = MIN_POS_HP - POS_OFFSET

MAX_POS_KN = 0.45
MIN_POS_KN = 0.16
MID_POS_KN = (MAX_POS_KN + MIN_POS_KN) / 2

MAX_POS_KN_SOFT_STOP = MAX_POS_KN + POS_OFFSET
MIN_POS_KN_SOFT_STOP = MIN_POS_KN + POS_OFFSET



async def main():
    # our system has 2 servos, each attached to a separate pi3hat bus
    # servo_bus_map argument describes which IDs are found on which bus
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:[11], # TODO: figure out which num corresponds to which bus port & joint
            2:[12],
        },
    )

    # We create one 'moteus.Controller' instance for each servo. It is not
    # strictly required to pass a 'transport' since we do not intend to use
    # any 'set_*' methods, but it doesn't hurt.
    #
    # syntax is a python "dictionary comprehension"
    servos = {
        servo_id : moteus.Controller(id = servo_id, transport=transport)
        for servo_id in [11, 12]
    }

    # We will start by sending a 'stop' to all servos, in the event that any had a fault
    await transport.cycle([x.make_stop() for x in servos.values()])

    while True:
        # the 'cycle' method accepts a list of commands, each of which is created by
        # calling one of the 'make_foo' methods on Controller. The most command thing
        # will be the 'make_position' method

        now = time.time()

        # construct a pos command for each servo, each of which consists of a
        # sinusoidal velocity command starting from wherever the servo was at
        # to begin with
        #
        # 'make_position' accepts optional keyword arguments that correspond
        # to each of the available position mode registers in the moteus
        # reference manual
        commands_sinusoidal = [
            servos[11].make_position(
                position = math.nan,
                velocity = 0.1 * math.sin(now),
                query = True),
            servos[12].make_position(
                position = math.nan,
                velocity = 0.1 * math.sin(now + 1),
                query = True),
        ]

        # position commands
        commands_pos = [
            servos[11].make_position( # assuming servos[11] is hip
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = MIN_POS_HP,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
            servos[12].make_position( # assuming servos[12] is knee
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = MIN_POS_KN,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
        ]

        # By sending all commands to the transport in one go, the pi3hat
        # can send out commands and retrieve responses simultaneously
        # from all ports. It can also pipeline commands and responses
        # for multiple servos on the same bus
        results = await transport.cycle(commands_sinusoidal)

        # The result is a list of 'moteus.Result' types, each of which
        # identifies the servo it came from, and has a 'values' field
        # that allows access to individual register results.
        #
        # NOTE: it is possible to not receive responses from all servos
        #       for which a query was requested
        #
        # Here, we'll just print the ID, position, and velocity of each
        # servo for which a reply was returned
        print(", ".join(
            f"({result.id}) " +
            f"{result.values[moteus.Register.POSITION]} " +
            f"{result.values[moteus.Register.VELOCITY]}"
            for result in results
        ))

        # We will wait 20ms between cycles. By default, each servo has
        # a watchdog timeout, where if no CAN command is received for
        # 100mc the controller will enter a latched fault state
        await asyncio.sleep(0.02)


if __name__ == '__main__':
    asyncio.run(main())
