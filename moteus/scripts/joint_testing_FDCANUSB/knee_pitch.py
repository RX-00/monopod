#!/usr/bin/python3 -B

# Python script to control and test the knee-pitch motor of the monopod
# High torque bandwidth mode for the moteus controller since this joint
# will experience large load external disturbances and will need to react
# very quickly.
#
# From the moteus docs, "it is recommended to have no integrative term in the position controller"
#
# TODO: increase the bandwidth through servo.pid_dq.kp/ki for testing
#       purposes for now

import asyncio
import math
import moteus
import time
import argparse
import sys

# TODO: Figure out the wrapped positions

# NOTE: ALWAYS USE THE WRAPPED POSITIONS FOR KNEE PITCH MOTOR
# wrapped pos with neg
MAX_POS_W = -0.15  # same as 1.00
MIN_POS_W = -0.65  # same as 0.50
MID_POS_W = (MAX_POS_W - MIN_POS_W) / 2.0 # same as 0.75

class knee_pitch:
    def __init__(self):
        #self.args = args

        # for now, let's just use the default query res of F32
        qr = moteus.QueryResolution()

        # kn_motor = knee pitch motor
        self.kn_motor = moteus.Controller(id = 1)
        self.pos_tolerance = 0.0


    async def stop(self):
        # stopping the knee pitch motor, TURNS OFF POWER
        await self.kn_motor.set_stop()

    async def init(self):
        await self.stop() # stops all the motors (but in this case it's just kn for now)
        #await self.starting_pos_check()

    async def starting_pos_check(self):
        while True:
            # update the current position to the closest one which is consistent with a output position
            await self.kn_motor.set_rezero(0.0, query = True)

            result = None
            while result is None:
                result = await self.kn_motor.query()

            if ((result.values[moteus.Register.POSITION] > MAX_POS_W + self.pos_tolerance) or
               (result.values[moteus.Register.POSITION] < MIN_POS_W + self.pos_tolerance)):
               print("OUTSIDE KNEE PITCH MOTOR SAFETY RANGE")
               print("Please manually move motors into correct range")
               print(result.values[moteus.Register.POSITION])
               self.stop()

            if ((result.values[moteus.Register.POSITION] <= MAX_POS_W + self.pos_tolerance) and
               (result.values[moteus.Register.POSITION] >= MIN_POS_W + self.pos_tolerance)):
                print("Knee Pitch Motor within safety range")
                await asyncio.sleep(2)
                break

        self.start_pos = result.values[moteus.Register.POSITION]
        print(self.start_pos)

    async def move_to_max_pos(self):
        # move motor from the current position to the max position
        while True:
            result = await self.kn_motor.set_position(
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = MAX_POS_W,
                feedforward_torque = 0.0,
                watchdog_timeout = math.nan,
                query = True)
            print(result.values[moteus.Register.POSITION])
            if result and result.values[moteus.Register.POSITION] >= MAX_POS_W:
                break

    async def move_to_min_pos(self):
        # move motor from the current position to the min position
        while True:
            result = await self.kn_motor.set_position(
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = MIN_POS_W,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True)
            print(result.values[moteus.Register.POSITION])
            if result and result.values[moteus.Register.POSITION] <= MIN_POS_W:
                break

    async def move_to_mid_pos(self):
        # move motor from the current position to the mid position
        while True:
            result = await self.kn_motor.set_position(
                position = math.nan,
                velocity = -2.0,
                maximum_torque = 2.0,
                stop_position = MID_POS_W,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True)
            print(result.values[moteus.Register.POSITION])
            if ((result.values[moteus.Register.POSITION] <= MID_POS_W + self.pos_tolerance) and
               (result.values[moteus.Register.POSITION] >= MID_POS_W - self.pos_tolerance)):
                break

async def main():
    print("Starting Knee Pitch Motor...")
    kn_joint = knee_pitch()

    await kn_joint.init()

    await kn_joint.move_to_max_pos()
    await asyncio.sleep(2)

    await kn_joint.move_to_min_pos()
    await asyncio.sleep(2)

    await kn_joint.move_to_mid_pos()
    await asyncio.sleep(2)

    await kn_joint.stop()
    sys.exit("Finished")


if __name__ == '__main__':
    asyncio.run(main())
