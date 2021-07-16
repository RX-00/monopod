#!/usr/bin/python3 -B

import asyncio
import math
import moteus
import time
import argparse
import sys

MAX_POS_W_HP = 0.00  # same as 1.00
MIN_POS_W_HP = -0.5  # same as 0.50
MID_POS_W_HP = -0.25 # same as 0.75

MAX_POS_W_KN = 0.00  # same as 1.00
MIN_POS_W_KN = -0.5  # same as 0.50
MID_POS_W_KN = -0.25 # same as 0.75

class Servo:
    def __init__(self, id):
        self.id = id
        self.controller = moteus.Controller(id = id)
        self.stream = moteus.Stream(self.controller)

    # Use the moteus stream for the unwrapped position
    async def read_unwrapped_position(self):
        servo_stats = await self.stream.read_data("servo_stats")
        return servo_stats.unwrapped_position

    # Use the moteus read/write registers for the wrapped position
    async def read_wrapped_position(self):
        result = None
        while result is None:
            result = await self.controller.query()
        return result.values[moteus.Register.POSITION]

    async def flush_read(self):
        await self.stream.flush_read()

    async def stop(self):
        # stops the motor and turns off power sent to the coils
        await self.controller.set_stop()

    async def set_configs(self, min_pos, max_pos, unw_pos_scale, pid_pos_ki):
        servo_stats = await self.stream.read_data("servo_stats")
        print(servo_stats.unwrapped_position)
        self.min_pos = min_pos
        self.max_pos = max_pos

        #await self.stream.command("conf set servopos.position_min {min_pos.5f}".encode('latin1'))

    async def move_to_max_pos(self):
        # move motor from the current position to the max position
        while True:
            result = await self.controller.set_position(
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = self.max_pos,
                feedforward_torque = 0.0,
                watchdog_timeout = math.nan,
                query = True)
            #print(result.values[moteus.Register.POSITION])
            if result and result.values[moteus.Register.POSITION] >= self.max_pos:
                break

    async def move_to_min_pos(self):
        # move motor from the current position to the min position
        while True:
            result = await self.controller.set_position(
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = self.min_pos,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True)
            #print(result.values[moteus.Register.POSITION])
            if result and result.values[moteus.Register.POSITION] <= self.min_pos:
                break



async def main():
    print("Starting Motors...")
    print("Testing hip pitch motor...")
    hp_joint = Servo(1) # currently no need for transport until rpi integration?

    print("flushing moteus stream...")
    await hp_joint.flush_read()

    # TODO: TEST IF SET_CONFIGS DOES WRAPPED OR UNWRAPPED POSITIONS
    await hp_joint.set_configs(MIN_POS_W_HP, MAX_POS_W_HP, 1, 0)
    print("configs set!")

    print("moving to max pos...")
    await hp_joint.move_to_max_pos()
    await asyncio.sleep(1.5)

    print("moving to min pos...")
    await hp_joint.move_to_min_pos()
    await asyncio.sleep(1.5)

    await hp_joint.stop()

    sys.exit("Finished")


if __name__ == '__main__':
    asyncio.run(main())
