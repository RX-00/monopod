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

Just rewrite this to be more like gen_motor_pos_pihat.py

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
        await self.controller.make_stop()

    async def stream_read_position(self):
        servo_stats = await self.stream.read_data("servo_stats")
        return servo_stats.unwrapped_position

    async def stream_zero_offset(self):
        servo_stats = await self.stream.read_data("servo_stats")
        position_raw = servo_stats.position_raw
        await self.stream.command(
            f"conf set motor.position_offset {-position_raw:d}".encode('latin1'))
        await self.stream.command("conf write".encode('latin1'))
        await self.stream.command("d rezero".encode('latin1'))

    async def stream_flush_read(self):
        await self.stream.flush_read()



async def main():
    servo_ids = {
        1:[1],
        2:[2],
    }

    print("creating moteus pi3hat transport")
    # our system has 2 servos, each attached to a separate pi3hat bus
    # servo_bus_map argument describes which IDs are found on which bus
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:[1], # KNEE
            2:[2], # HIP
        },
    )

    servos = [Servo(transport, x) for x in servo_ids]

    # We will start by sending a 'stop' to all servos, in the event that any had a fault
    #await transport.cycle()
    print("sent stop cmd to clear any motor faults")

    half_kn = (MAX_POS_KN - MIN_POS_KN) / 2
    half_hp = (MAX_POS_HP - MIN_POS_HP) / 2

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
            servos[1].controller.make_position( # KNEE
                position = math.nan,
                velocity = 0.5,
                maximum_torque = 2.0,
                stop_position = MIN_POS_KN + half_kn + math.sin(now) * half_kn,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
            servos[2].controller.make_position( # HIP
                position = math.nan,
                velocity = 0.5,
                maximum_torque = 2.0,
                stop_position = MIN_POS_HP + half_hp + math.sin(now + 1) * half_hp,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
        ]

        # position commands
        commands_pos = [
            servos[1].controller.make_position( # KNEE
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = MAX_POS_KN,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
            servos[2].controller.make_position( # HIP
                position = math.nan,
                velocity = 2.0,
                maximum_torque = 2.0,
                stop_position = MAX_POS_HP,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True),
        ]

        # By sending all commands to the transport in one go, the pi3hat
        # can send out commands and retrieve responses simultaneously
        # from all ports. It can also pipeline commands and responses
        # for multiple servos on the same bus
        #results = await transport.cycle(commands_sinusoidal)
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
        #print("\n".join(
        #    f"({result.id}) " +
        #    f"{result.values[moteus.Register.POSITION]} " +
        #    f"{result.values[moteus.Register.VELOCITY]} "
        #    for result in results
        #), end='\r')
        print(now, end='\r')

        # We will wait 20ms between cycles. By default, each servo has
        # a watchdog timeout, where if no CAN command is received for
        # 100mc the controller will enter a latched fault state
        await asyncio.sleep(0.02)



if __name__ == '__main__':
    asyncio.run(main())
