# motion planning and controlling module
# the goal of this module is to give the environment variables,
# send out control signals to the car

import sys
import io
import time
import logging
import numpy as np
import enum

# class for motion task states
class TaskStates(enum.Enum):
    FindBarrel = 1
    CatchBarrel = 2
    MoveBarrel = 3
    UnloadBarrel = 4

class MotionPlanning:


    






