#!/usr/bin/python3 -B

# Copyright 2020 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import math
import moteus
import time

async def sine_wave():
    c = moteus.Controller()

    for i in range(2000):
        print(await c.set_position(position = 0.9 * math.sin(math.pi*i/500), query = True))
        await asyncio.sleep(0.001)

    print(await c.set_stop(query = True))    # in case there was a fault


async def one_full_rotation():
    c = moteus.Controller()

    for i in range(500):
        print(await c.set_position(position = abs(0.9999 * math.sin(math.pi*i/500)), query = True))
        await asyncio.sleep(0.001)

    print(await c.set_stop(query = True))    # in case there was a fault


async def main():
    c = moteus.Controller()
    await c.set_stop()  # in case there was a fault

    while True:
        print(await c.set_position(position=math.nan, query=True))
        await asyncio.sleep(0.02)

if __name__ == '__main__':
    #asyncio.run(main())
    #asyncio.run(sine_wave())
    asyncio.run(one_full_rotation())
