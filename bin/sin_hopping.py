#!/usr/bin/python3 -B

# Sinusoidal jumping program for the 2D monopod setup

from ctrlrs.ik.sin_ik_hop_ctrlr import sinIkHopCtrlr

import asyncio
import math
import moteus
import time
import argparse
import sys



if __name__ == "__main__":
    ctrlr = sinIkHopCtrlr(25.0, 0.015, 1.0, 1.2, False)
