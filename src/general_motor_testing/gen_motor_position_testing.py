#!/usr/bin/python3 -B

# Python script to control and test both of the hip-pitch &  knee-pitch
# motor of the monopod
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

# NOTE: ALWAYS USE THE WRAPPED POSITIONS FOR HIP PITCH MOTOR
# wrapped pos with neg
MAX_POS_W_HP = 0.00  # same as 1.00
MIN_POS_W_HP = -0.5  # same as 0.50
MID_POS_W_HP = -0.25 # same as 0.75

MAX_POS_W_KN = 0.00  # same as 1.00
MIN_POS_W_KN = -0.5  # same as 0.50
MID_POS_W_KN = -0.25 # same as 0.75

class Servo:
    def __init__(self, transport, id):
        self.id = id
        self.transport = transport
        self.controller = moteus.Controller(id = id, transport = transport)
        self.stream = moteus.Stream(self.controller) # allows python file-like interface to the diagnostic stream of a moteus controller
        # NOTE: TODO: perhaps force set_limits in the constructor function

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

    # NOTE: pid_pos_ki should be set to 0 for high torque bandwidth mode
    # NOTE: unw_pos_scale should be 1 if no gearbox reduction or multiplier
    async def set_configs(self, min_pos, max_pos, unw_pos_scale, pid_pos_ki):
        servo_stats = await self.stream.read_data("servo_stats")
        self.min_pos = min_pos
        self.max_pos = max_pos

        # {:d} for decimal integer in base 10
        await self.stream.command("conf set servopos.position_min {min_pos:d}".encode('latin1'))
        await self.stream.command("conf set servopos.position_max {max_pos:d}".encode('latin1'))
        await self.stream.command("conf set motor.unwrapped_position_scale {unw_pos_scale:d}".encode('latin1'))
        await self.stream.command("conf set servo.pid_position.ki {pid_pos_ki:d}".encode('latin1'))

    # NOTE: with the above servopos.position_min/max we shouldn't need a
    #       starting_pos_check as seen in src/joint_testing

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
            print(result.values[moteus.Register.POSITION])
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
            print(result.values[moteus.Register.POSITION])
            if result and result.values[moteus.Register.POSITION] <= self.min_pos:
                break

    async def move_to_mid_pos(self):
        # move motor from the current position to the mid position
        mid_pos = (self.max_pos + self.min_pos) / 2
        while True:
            result = await self.controller.set_position(
                position = math.nan,
                velocity = -2.0,
                maximum_torque = 2.0,
                stop_position = mid_pos,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True)
            print(result.values[moteus.Register.POSITION])
            if ((result.values[moteus.Register.POSITION] <= mid_pos) and
               (result.values[moteus.Register.POSITION] >= mid_pos)):
                break

async def main():
    print("Starting Motors...")
    print("Testing hip pitch motor...")
    hp_joint = Servo(transport, 1) # currently no need for transport until rpi integration?

    # TODO: TEST IF SET_CONFIGS DOES WRAPPED OR UNWRAPPED POSITIONS
    await hp_joint.set_configs(MIN_POS_W, MAX_POS_W, 1, 0)

    await hp_joint.move_to_max_pos()
    await asyncio.sleep(1.5)
    await hp_joint.move_to_mid_pos()
    await asyncio.sleep(1.5)
    await hp_joint.move_to_min_pos()

    hp_joint.stop()
    sys.exit("Finished")


if __name__ == '__main__':
    asyncio.run(main())
