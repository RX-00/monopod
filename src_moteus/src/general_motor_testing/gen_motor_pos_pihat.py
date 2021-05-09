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
        servo_id : moteus.Controller(id = servo_id, transport transport)
        for servo_id in [11, 12]
    }

    # We will start by sending a 'stop' to all servos, in the event that any had a fault
    await transport.cycle([x.make_stop() for x in servos.values()])

    while True:
        # the 'cycle' method accepts a list of commands, each of which is created by
        # calling one of the 'make_foo' methods on Controller. The most command thing
        # will be the 'make_position' method

        now = time.time()



if __name__ == '__main__':
    asyncio.run(main())
