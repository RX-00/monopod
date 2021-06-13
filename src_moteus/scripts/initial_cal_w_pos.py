#!/usr/bin/python3 -B

# Python script to calibrate and set configurable parameters
# position_min/max along with printing out the current read
# positions of the motors for the monopod

import asyncio
import math
import moteus
import moteus_pi3hat
import time
import argparse
import sys


'''
TODO:

Program Layout:
    - does not send all the commands in one go
    - instead have a prompt to ask which controller id's to start with
        - prompt to enter number of id you wish to test with
    - start to print out the position of the motor id selected (no scrolling)
    - ask to move to max position and then press a key to confirm to set max position
    - ask to move to min position and then press a key to confirm to set min position
    - exit the position calibration and print out the stored max/min positions and
      ask if you want to confirm and save these positions [y/n]
    - exit the program
'''



class Servo:
    def __init__(self, id):
        self.id = id
        self.controller = moteus.Controller(id = id)
        self.stream = moteus.Stream(self.controller) # moteus diagnostic stream, do NOT use for real time

    async def read_stream_pos(self): # not good for real time
        servo_stats = await self.stream.read_data("servo_stats")
        return servo_stats.unwrapped_position

    # Use the moteus read/write registers for the wrapped position
    async def read_unwrapped_position(self):
        result = None
        while result is None:
            result = await self.controller.query()
        return result.values[moteus.Register.POSITION]

    # Use the moteus read/write registers for current mode
    async def read_current(self):
        result = None
        while result is None:
            result = await self.controller.query()
        q_current_val = result.values[moteus.Register.COMMAND_Q_CURRENT]
        print(q_current_val)

    # Use the moteus read/write registers for fault mode
    async def read_fault(self):
        result = None
        while result is None:
            result = await self.controller.query()
        fault_val = result.values[moteus.Register.FAULT]
        print(fault_val)

    async def flush_read(self): # to flush out the past command(s) on the moteus
        await self.stream.flush_read()

    async def print_unwrapped_position(self, oneLine):
        result = None
        while result is None:
            result = await self.controller.query()
        if oneLine:
            print(result.values[moteus.Register.POSITION], end='\r')
        elif not oneLine:
            print(result.values[moteus.Register.POSITION])

    async def stop(self):
        # stops the motor and turns off power sent to the coils
        await self.controller.set_stop()

    # NOTE: from mjbots/quad/.../zero_leg.py for reference
    async def zero_offset(self):
        servo_stats = await self.stream.read_data("servo_stats")
        position_raw = servo_stats.position_raw
        await self.stream.command(
            f"conf set motor.position_offset {-position_raw:d}".encode('latin1'))
        await self.stream.command("conf write".encode('latin1'))
        await self.stream.command("d rezero".encode('latin1'))

    # NOTE: pid_pos_ki should be set to 0 for high torque bandwidth mode
    # NOTE: unw_pos_scale should be 1 if no gearbox reduction or multiplier
    async def set_configs(self, unw_pos_scale, pid_pos_ki):
        servo_stats = await self.stream.read_data("servo_stats")
        print(servo_stats.unwrapped_position)
        self.curr_pos = servo_stats.unwrapped_position

        print("This is your min_pos value: ", self.min_pos)
        print("This is your max_pos value: ", self.max_pos)

        cmd_str = "conf set motor.unwrapped_position_scale " + str(unw_pos_scale)

        # NOTE: No firmware min/max position bounds due to the fault it results in during runtime
        await self.stream.command(cmd_str.encode('latin1'))
        await self.stream.command("conf set servo.pid_position.ki {pid_pos_ki:.5f}".encode('latin1'))

    async def set_min_pos(self, pos_val):
        self.min_pos = pos_val

    async def set_max_pos(self, pos_val):
        self.max_pos = pos_val

    async def obtain_cal_pos_limit(self, min_or_max):
        pos_val = 0
        print("please move motor to the " + min_or_max + " position limit")
        print("press [q] to set or any other key to continue")
        while True:
            await self.print_unwrapped_position(True)
            pos_val = await self.read_unwrapped_position()
            n = input()
            if n == 'q':
                break


        print(min_or_max, " position has been set as: ", pos_val)
        print("now setting the " + min_or_max + " limit")

        if min_or_max == "min":
            await self.set_min_pos(pos_val)
        if min_or_max == "max":
            await self.set_max_pos(pos_val)
        print("motor limit has been saved")


async def main():
    print("Starting motor calibrations...")

    # Servo 1 calibration ============================================
    servo1 = Servo(1)
    print("clearing any faults by sending stop command...")
    await servo1.stop()

    min_or_max = "min"
    await servo1.obtain_cal_pos_limit(min_or_max)
    min_or_max = "max"
    await servo1.obtain_cal_pos_limit(min_or_max)

    await servo1.set_configs(1, 0)
    await servo1.stop()


    # Servo 2 calibration ============================================
    print("\n\n\n Next servo calibration...")
    servo2 = Servo(2)
    print("Clearing any faults by sending stop command...")
    await servo2.stop()

    min_or_max = "min"
    await servo2.obtain_cal_pos_limit(min_or_max)
    min_or_max = "max"
    await servo2.obtain_cal_pos_limit(min_or_max)

    await servo2.set_configs(1, 0)
    await servo2.stop()


    print("Servo calibrations have been set and saved! \n exiting program...")
    sys.exit("See ya!")



if __name__ == '__main__':
    asyncio.run(main())
