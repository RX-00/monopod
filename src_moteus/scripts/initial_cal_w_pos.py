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



async def main():

    print("creating moteus pi3hat transport")
    # our system has 2 servos, each attached to a separate pi3hat bus
    # servo_bus_map argument describes which IDs are found on which bus
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:[1], # TODO: figure out which num corresponds to which bus port & joint
            2:[2],
        },
    )

    # We create one 'moteus.Controller' instance for each servo. It is not
    # strictly required to pass a 'transport' since we do not intend to use
    # any 'set_*' methods, but it doesn't hurt.
    #
    # syntax is a python "dictionary comprehension"
    servos = {
        servo_id : moteus.Controller(id = servo_id, transport=transport)
        for servo_id in [1, 2]
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
            servos[1].make_position(
                position = math.nan,
                velocity = 0.1 * math.sin(now),
                query = True),
            servos[2].make_position(
                position = math.nan,
                velocity = 0.1 * math.sin(now + 1),
                query = True),
        ]

        # position commands
        commands_pos = [
            servos[1].make_position( # assuming servos[11] is hip
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = MIN_POS_HP,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
            servos[2].make_position( # assuming servos[12] is knee
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
