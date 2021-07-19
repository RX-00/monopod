#!/usr/bin/python3 -B

# Jumping program for the 2D monoped setup
# This program should be extendable to add more servos (at later stages)
# such as, roll and yaw servos in the hip and also two ankle servos
# NOTE: ankle servos may end up being hobby servos (but this will be designed more in the future)
#

import asyncio
import math
import moteus
import moteus_pi3hat
import time
import argparse
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




class Leg:
    # each arg corresponds to the respective servo CANBUS ID
    async def __init__(self, knee, hip_pitch):
        self.knee = knee
        self.hip_pitch = hip_pitch

        # servo_bus_map arg describes which IDs are found on which bus
        self.transport = moteus_pi3hat.Pi3HatRouter(
            servo_bus_map = {
                1:[self.knee],
                2:[self.hip_pitch],
            },
        )
        # explicit servo_id's
        self.port_knee = 0
        self.port_hip_pitch = 1

        # create a moteus.Controller instance for each servo
        self.servos = {
            servo_id : moteus.Controller(id = servo_id, transport=self.transport)
            for servo_id in [1, 2] # number of motors, need to change manually
        }

        # commands for the motors, default set to mid values
        self.commands = [
            self.servos[self.knee].make_position(
                position = math.nan,
                velocity = 0.5,
                maximum_torque = 2.0,
                stop_position = (MAX_POS_KN - MIN_POS_KN) / 2,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True
            ),
            self.servos[self.hip_pitch].make_position(
                position = math.nan,
                velocity = 0.5,
                maximum_torque = 2.0,
                stop_position = (MAX_POS_HP - MIN_POS_HP) / 2,
                feedforward_torque = -0.01,
                watchdog_timeout = math.nan,
                query = True
            ),
        ]


    # send stop to clear faults && disable all the motors
    async def stop_all_motors(self):
        await self.transport.cycle([n.make_stop() for n in self.servos.values()])


    # set knee motor commands
    async def set_motor_kn_cmds(self, pos, vel, max_torq, stop_pos, ffwd_torq, watchdog_timeout, que):
        self.commands.insert(self.port_knee,
                             self.servos[self.knee].make_position(
                                 position = pos,
                                 velocity = vel,
                                 maximum_torque = max_torq,
                                 stop_position = stop_pos,
                                 feedforward_torque = ffwd_torq,
                                 watchdog_timeout = watchdog_timeout,
                                 query = que),
                             )


    # set hip motor commands
    async def set_motor_hp_cmds(self, pos, vel, max_torq, stop_pos, ffwd_torq, watchdog_timeout, que):
        self.commands.insert(self.port_hip_pitch,
                             self.servos[self.hip_pitch].make_position(
                                 position = pos,
                                 velocity = vel,
                                 maximum_torque = max_torq,
                                 stop_position = stop_pos,
                                 feedforward_torque = ffwd_torq,
                                 watchdog_timeout = watchdog_timeout,
                                 query = que),
                             )


    # send commands and return the results info
    async def send_motor_cmds(self):
        results = await self.transport.cycle(self.commands)
        return results




async def main():

    print("Starting jumping...")
    monopod = Leg(1, 2)
    # halfway position of each motorized joint
    kn_half = (MAX_POS_KN - MIN_POS_KN) / 2
    hp_half = (MAX_POS_HP - MIN_POS_HP) / 2

    # clearing any faults
    await monopod.stop_all_motors()
    print("cleared motor faults")

    while True:

        now = time.time()

        # testing the set commands
        await monopod.set_motor_kn_cmds(math.nan, 0.5, 2.0, MIN_POS_KN + kn_half + math.sin(now) * kn_half, -0.01, math.nan, True)
        await monopod.set_motor_hp_cmds(math.nan, 0.5, 2.0, MIN_POS_HP + hp_half + math.sin(now + 1) * hp_half, -0.01, math.nan, True)

        #print(monopod.commands[0])
        #print(monopod.commands[1])


        # test sending the commands
        results = await monopod.send_motor_cmds()

        # The result is a list of 'moteus.Result' types, each of which
        # identifies the servo it came from, and has a 'values' field
        # that allows access to individual register results.
        #
        # NOTE: it is possible to not receive responses from all servos
        #       for which a query was requested
        #
        # Here, we'll just print the ID, position, and velocity of each
        # servo for which a reply was returned
        '''print("".join(
            f"({result.id}) " +
            f"{result.values[moteus.Register.POSITION]} " +
            f"{result.values[moteus.Register.VELOCITY]} "
            for result in results
        ), end='\r')'''
        #print(now, end='\r')


        # await asyncio.sleep(0.5)



if __name__ == '__main__':
    asyncio.run(main())
















