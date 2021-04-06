#!/usr/bin/python3 -B

# Python script to control and test both of the hip-pitch &  knee-pitch
# motor of the monopod
# High torque bandwidth mode for the moteus controller since this joint
# will experience large load external disturbances and will need to react
# very quickly.
#
# From the moteus docs, "it is recommended to have no integrative term in the position controller"
#


'''
TODO: WRITE SERVO POS TORQUE TIME FUNCTION FOR FUTURE VIRTUAL LEG
      ALGORITHM FOR MONOPOD LEG TESTING


'''

import asyncio
import math
import moteus
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


class Servo:
    def __init__(self, id): # NOTE: instantiate your own transport instance when using rpi CAN-FD interface
        self.id = id
        self.controller = moteus.Controller(id = id)
        self.stream = moteus.Stream(self.controller) # moteus diagnostic stream, do NOT use for real time

    async def read_unwrapped_position_stream(self):
        servo_stats = await self.stream.read_data("servo_stats")
        return servo_stats.unwrapped_position

    # Use the moteus read/write registers for the wrapped position
    async def read_unwrapped_position(self):
        result = None
        while result is None:
            result = await self.controller.query()
        return result.values[moteus.Register.POSITION]

    # Use the moteus read/write registers for current mode
    async def read_currents(self):
        result = None
        while result is None:
            result = await self.controller.query()
        #q_current_val = result.values[moteus.Register.COMMAND_Q_CURRENT]
        #print(q_current_val)

    # Use the moteus read/write registers for fault mode
    async def read_fault(self):
        result = None
        while result is None:
            result = await self.controller.query()
        fault_val = result.values[moteus.Register.FAULT]
        print(fault_val)

    async def flush_read(self):
        await self.stream.flush_read()

    async def print_unwrapped_position(self):
        result = None
        while result is None:
            result = await self.controller.query()
        print(result.values[moteus.Register.POSITION])

    async def stop(self):
        # stops the motor and turns off power sent to the coils
        await self.controller.set_stop()

    # NOTE: pid_pos_ki should be set to 0 for high torque bandwidth mode
    # NOTE: unw_pos_scale should be 1 if no gearbox reduction or multiplier
    async def set_configs(self, min_pos_hs, max_pos_hs, min_pos, max_pos, unw_pos_scale, pid_pos_ki):
        servo_stats = await self.stream.read_data("servo_stats")
        print(servo_stats.unwrapped_position)
        self.curr_pos = servo_stats.unwrapped_position
        self.min_pos_hs = min_pos_hs
        self.max_pos_hs = max_pos_hs
        self.min_pos = min_pos
        self.max_pos = max_pos

        cmd_str = "conf set motor.unwrapped_position_scale " + str(unw_pos_scale)

        # No firmware min/max position bounds due to the fault it results in during runtime
        await self.stream.command(cmd_str.encode('latin1'))
        await self.stream.command("conf set servo.pid_position.ki {pid_pos_ki:.5f}".encode('latin1'))

    async def curr_pos_range_check(self):
        pos_check = true
        if self.curr_pos > self.max_pos_hs or self.curr_pos < self.min_pos_hs:
            pos_check = false
            sys.exit("ERROR: current position is out of range")
        return pos_check

    async def cmd_pos_range_check(self, cmd_pos):
        pos_check = true
        if cmd_pos > self.max_pos_hs or cmd_pos < self.min_pos_hs:
            pos_check = false
            print("ERROR: cmd position is out of range")
        return pos_check

    async def move_to_max_pos(self):
        # move motor from the current position to the max position
        while True:
            result = await self.controller.set_position(
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = self.max_pos,
                feedforward_torque = 0.0,
                watchdog_timeout = math.nan, # currently disabled for testing purposes
                query = True)
            #print(result.values[moteus.Register.POSITION])
            self.curr_pos = result.values[moteus.Register.POSITION]
            if result and result.values[moteus.Register.POSITION] >= self.max_pos_hs:
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
            self.curr_pos = result.values[moteus.Register.POSITION]
            if result and result.values[moteus.Register.POSITION] <= self.min_pos_hs:
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
            #print(result.values[moteus.Register.POSITION])
            self.curr_pos = result.values[moteus.Register.POSITION]
            if ((result.values[moteus.Register.POSITION] <= mid_pos + 0.01) and
               (result.values[moteus.Register.POSITION] >= mid_pos - 0.01)):
                break



async def main_hp_joint_test():
    print("Starting Motors...")
    print("Testing hip pitch motor...")
    hp_joint = Servo(1) # currently no need for transport until rpi integration?
    print("clear any faults by sending stop command")
    await hp_joint.stop()
    await hp_joint.set_configs(MIN_POS_HP_SOFT_STOP, MAX_POS_HP_SOFT_STOP, MIN_POS_HP, MAX_POS_HP, 1, 0)
    print("configs set!")
    print("moving to max pos...")
    await hp_joint.move_to_max_pos()
    await asyncio.sleep(.5)
    print("reading currents...")
    await hp_joint.read_currents()
    print("reading fault...")
    await hp_joint.read_fault() # if 0 = no faults?
    print("moving to mid pos...")
    await hp_joint.move_to_mid_pos()
    await asyncio.sleep(.5)
    print("moving to min pos...")
    await hp_joint.move_to_min_pos()
    await asyncio.sleep(.5)
    print("moving to max pos...")
    await hp_joint.move_to_max_pos()
    await asyncio.sleep(.5)
    await hp_joint.stop()
    sys.exit("Finished")

async def main_kn_joint_test():
    print("Starting Motors...")
    print("Testing knee pitch motor...")
    kn_joint = Servo(1) # currently no need for transport until rpi integration?
    print("clear any faults by sending stop command")
    await kn_joint.stop()
    await kn_joint.set_configs(MIN_POS_KN_SOFT_STOP, MAX_POS_KN_SOFT_STOP, MIN_POS_KN, MAX_POS_KN, 0.588, 0)
    print("configs set!")
    #while True:
    #    await kn_joint.print_unwrapped_position()
    print("moving to max pos...")
    await kn_joint.move_to_max_pos()
    await asyncio.sleep(0.5)
    print("moving to mid pos...")
    await kn_joint.move_to_mid_pos()
    await asyncio.sleep(0.5)
    print("moving to min pos...")
    await kn_joint.move_to_min_pos()
    await asyncio.sleep(0.5)
    print("moving to max pos...")
    await kn_joint.move_to_max_pos()
    await asyncio.sleep(0.5)
    await kn_joint.stop()
    sys.exit("Finished")


if __name__ == '__main__':
    asyncio.run(main_kn_joint_test())
