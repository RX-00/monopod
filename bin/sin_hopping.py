#!/usr/bin/python3 -B

# Sinusoidal jumping program for the 2D monopod setup

from ctrlrs.ik.sin_ik_hop_ctrlr import sinIkHopCtrlr


if __name__ == "__main__":
    ctrlr = sinIkHopCtrlr(True)
    ctrlr.sinusoidal_mv()
