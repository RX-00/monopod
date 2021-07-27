# monopod
WIP 2D monopod

SETUP VENV:
 1. If on a new platform w/o monopod/env then: python3 -m venv /path/to/new/virtual/environment
 2. don't forget to 'source monopod/env/bin/activate'
 3. then in the (env) pip3 install -e monopod
 4. don't forget to use setup.py w/ 'pip3 install .'

NOTE:
 - if the knee motor is not working despite no changes in hw and sw then do:
   1. complete power cycle
   2. run the move knee script
   3. retry
 - velocity command should be 0.0 if not specified, NOT math.nan!!

TODO:
- test out lcm py and cpp
    - create a make file for lcm
- maybe as the project evolves, use catkin to manage and build the project
- ADD AN ENCODER TO KNEE AND USE NEW FIRMWARE FEATURE ON MOTEUS CONTROLLER FOR REDUCTION UP GEAR RATIO
- set servo.position_min/max for moteus on lower level using command line tools or python diagnostic stream


BOM leg:
- moteus controller + test aluminium mount
- moteus brushless motor
- pi3hat for pi4 from mjbots
- pwr dist board r4.4 from mjbots
- TOPPROS 120XL SeriesWidth 3/8" 60 Teeth
- M3, M5 countersunk screws
- uxcell 6902-2RS Deep Groove Ball Bearing 15x28x7mm Double Sealed ABEC-3 Bearings

BOM testing rig:
- 2020 aluminium extrusion (100mm, 500mm)
- 2020 aluminium extrusion connectors, L brackets, screws, extension connectors, etc.
- 2020 linear ball bearing slider
