# motion planning and controlling module
# the goal of this module is to give the environment variables,
# send out control signals to the car

import sys
import io
import time
import logging
import numpy as np
import enum
import threading

from Motor import Motor


# class for motion task states
class TaskStates(enum.Enum):
    FindBarrel = 1
    CatchBarrel = 2
    MoveBarrel = 3
    UnloadBarrel = 4


class MotionPlanning:
    def __init__(self):
        self.task_state = TaskStates.FindBarrel
        self.car = Motor()
        self.car_timer = None

        self.param_forward_speed = 800
        self.param_turn_speed = 1600

    def step(self, arena_floor, time_out=0):
        if self.car_timer is not None:
            self.car_timer.cancel()

        if self.task_state == TaskStates.FindBarrel:
            self.barrel_finding(arena_floor)
        elif self.task_state == TaskStates.CatchBarrel:
            self.barrel_catching(arena_floor)
        elif self.task_state == TaskStates.MoveBarrel:
            self.barrel_moving(arena_floor)
        elif self.task_state == TaskStates.UnloadBarrel:
            self.barrel_unloading(arena_floor)

        if time_out > 0:
            self.car_timer = threading.Timer(time_out, self.car.stop)
            self.car_timer.start()

    def barrel_finding(self, arena_floor):
        if len(arena_floor.red_barrels_in_view) == 0 and len(arena_floor.green_barrels_in_view) == 0:
            self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
        elif arena_floor.barrel_in_center_zone:
            self.task_state = TaskStates.CatchBarrel
            self.car.move_forward(self.param_forward_speed)
        else:
            min_red_dist, min_red_barrel = self.min_dist_to_center(arena_floor.red_barrels_in_view)
            min_green_dist, min_green_barrel = self.min_dist_to_center(arena_floor.green_barrels_in_view)
            if min_red_dist < min_green_dist:
                min_dist = min_red_dist
                min_barrel = min_red_barrel
            else:
                min_dist = min_green_dist
                min_barrel = min_green_barrel
            if min_barrel[5] > arena_floor.img_width:
                self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
            else:
                self.car.turn_left(self.param_turn_speed, self.param_turn_speed)

    def min_dist_to_center(self, barrels, img_width, img_height):
        min_dist = img_width
        min_barrel = []
        if len(barrels) > 0:
            for barrel in barrels:
                dist_l = np.abs(barrel[0] - img_width / 2)
                dist_r = np.abs(barrel[0] + barrel[2] - img_width / 2)
                dist = np.min(dist_l, dist_r)
                if dist < min_dist:
                    min_dist = dist
                    min_barrel = barrel
        return min_dist, min_barrel

