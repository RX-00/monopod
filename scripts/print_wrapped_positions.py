#!/usr/bin/python3 -B

# Python script to print the current wrapped positions
# of a connected moteus controller and motor

import asyncio
import math
import moteus
import time
import argparse
import sys

class motor:
    def __init__(self):
        #self.args = args

        # for now, let's just use the default query res of F32
        qr = moteus.QueryResolution()

        # hp_motor = hip pitch motor
        self.m_ctrlr = moteus.Controller(id = 1)
        self.pos_tolerance = 0.05


    async def stop(self):
        # stopping the hip pitch motor, TURNS OFF POWER
        await self.m_ctrlr.set_stop()

    async def init(self):
        await self.stop() # stops all the motors (but in this case it's just hp for now)
        await self.starting_pos_check()

    async def starting_pos_check(self):
        while True:
            # update the current position to the closest one which is consistent with a output position
            await self.m_ctrlr.set_rezero(0.0, query = True)

            result = None
            while result is None:
                result = await self.m_ctrlr.query()

            print("unwrapped motor pos: ", result.values[moteus.Register.POSITION])
            self.stop()

        self.start_pos = result.values[moteus.Register.POSITION]
        print(self.start_pos)


async def main():
    print("Reading and printing wrapped motor positions...")
    test_motor = motor()

    await test_motor.init()

    await test_motor.stop()
    sys.exit("Finished")


if __name__ == '__main__':
    asyncio.run(main())
