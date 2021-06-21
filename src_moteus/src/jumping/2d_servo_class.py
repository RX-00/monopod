#!/usr/bin/env python3

# Jumping program for the 2D monoped setup
# This program should be extendable to add more servos (at later stages)
# such as, roll and yaw servos in the hip and also two ankle servos
#
# NOTE: This implementation is meant to speed up the commands sent to moteus ctrlr
#       Especially compared to the og 2d_leg_class.py implementation
#

import math
import time
import argparse
import asyncio
import moteus
import moteus_pi3hat
import sys


'''
TODO:

REDO THIS WHOLE GODDAMN THING TO BE MORE LIKE ZERO_LEG.PY

Figure out how to make the spring jumping up and down virtual leg
1. get the measurements for the leg links
2. figure out the lowest possible length of the virtual leg spring
3. then figure out the IK that needs to be mapped to the input positions of the servos
   (This will be implemented as a cpp ctrlr that will interface with this py class)

'''


'''
NOTE: On the moteus controller these are the set position_min/max

KNEE servopos.position_min = -0.65
KNEE servopos.position_max = -0.15

HIP servopos.position_min = -0.51
HIP servopos.position_max = +0.02

KNEE id.id = 1
HIP  id.id = 2
'''


MAX_POS_KN = -0.15 - 0.00
MIN_POS_KN = -0.65 + 0.00

MAX_POS_HP =  0.02 - 0.00
MIN_POS_HP = -0.51 + 0.00





class Servo:
    def __init__(self, transport, id):
        self.id = id
        self.transport = transport
        self.controller = moteus.Controller(id=id, transport=transport)
        self.stream = moteus.Stream(self.controller)

    async def read_position(self):
        servo_stats = await self.stream.read_data("servo_stats")
        return servo_stats.unwrapped_position

    async def zero_offset(self):
        servo_stats = await self.stream.read_data("servo_stats")
        position_raw = servo_stats.position_raw
        await self.stream.command(
            f"conf set motor.position_offset {-position_raw:d}".encode('latin1'))
        await self.stream.command("conf write".encode('latin1'))
        await self.stream.command("d rezero".encode('latin1'))

    async def flush_read(self):
        await self.stream.flush_read()


async def async_readline(fd):
    result = b''
    while True:
        this_char = await fd.read(1)
        if this_char == b'\n' or this_char == b'\r':
            return result
        result += this_char


async def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        '-l', '--leg', type=int, help='leg (1-4) to zero')

    args = parser.parse_args()
    servo_ids = {
        1: [1, 2, 3],
        2: [4, 5, 6],
        3: [7, 8, 9],
        4: [10, 11, 12],
    }[args.leg]

    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1: [1, 2, 3],
            2: [4, 5, 6],
            3: [7, 8, 9],
            4: [10, 11, 12],
        },
    )
    servos = [Servo(transport, x) for x in servo_ids]
    [await servo.flush_read() for servo in servos]

    aio_stdin = moteus.aiostream.AioStream(sys.stdin.buffer.raw)
    read_future = asyncio.create_task(async_readline(aio_stdin))

    while True:
        print(', '.join([f"{servo.id: 2d}: {await servo.read_position():7.3f}"
                       for servo in servos]) + '    ',
              end='\r', flush=True)
        if read_future.done():
            break

    print()
    print()
    print("Zeroing servos")

    [await servo.zero_offset() for servo in servos]

    print("DONE")


if __name__ == '__main__':
    asyncio.run(main())
