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
        self.param_speed_adjust = 0.1
        self.param_sleep_time = 0.2

        self.param_img_ctr_offset = 0.1


    def step(self, arena_floor, time_out=0):
        # cancel the time out thread
        if self.car_timer is not None:
            self.car_timer.cancel()
        # motion planning for each state
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

    # find a barrel to move
    def barrel_finding(self, arena_floor):
        if len(arena_floor.red_barrels_in_view) == 0 and len(arena_floor.green_barrels_in_view) == 0:
            self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
        elif len(arena_floor.center_barrel) > 0:  # barrel at image center, move to next state
            self.task_state = TaskStates.CatchBarrel
            self.barrel_catching(arena_floor)
        else:
            min_red_dist, min_red_barrel = self.min_dist_to_center(arena_floor.red_barrels_in_view)
            min_green_dist, min_green_barrel = self.min_dist_to_center(arena_floor.green_barrels_in_view)
            if min_red_dist < min_green_dist:
                min_dist = min_red_dist
                min_barrel = min_red_barrel
            else:
                min_dist = min_green_dist
                min_barrel = min_green_barrel
            if min_barrel[5] > arena_floor.img_width / 2:
                self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
            else:
                self.car.turn_left(self.param_turn_speed, self.param_turn_speed)

    def barrel_catching(self, arena_floor):
        if arena_floor.barrel_in_arm:  # barrel is already in arm
            # move to the next state
            self.task_state = TaskStates.MoveBarrel
            self.barrel_moving(arena_floor)
        else:
            if len(arena_floor.center_barrel) > 0:  # barrel at center, but not in arm yet
                # move toward the barrel, but also correcting the error
                dist_err = arena_floor.center_barrel[5]/arena_floor.img_width - 0.5
                speed_adjust_ratio = np.abs(dist_err) / self.param_img_ctr_offset * self.param_speed_adjust
                adjust_speed = self.param_forward_speed * (1 - speed_adjust_ratio)
                if dist_err > 0:  # the target is to the right of the center
                    # move forward with right wheels turn a bit slower (turn right slightly)
                    self.car.move_forward(self.param_forward_speed, adjust_speed)
                elif dist_err < 0:  # the target is to the right of the right of the center
                    # move forward with left wheels turn a bit slower (turn left slightly)
                    self.car.move_forward(adjust_speed, self.param_forward_speed)
                else:
                    # move forward with same speed
                    self.car.move_forward(self.param_forward_speed)
            else:  # barrel is not at the center any more
                # move back to previous state
                self.task_state = TaskStates.FindBarrel
                self.barrel_finding(arena_floor)

    # moving barrels to the zone
    def barrel_moving(self, arena_floor):
        if not arena_floor.barrel_in_arm:
            # move back to previous state
            self.task_state = TaskStates.CatchBarrel
            self.barrel_catching(arena_floor)
        else:
            # determine which zone to move to
            if arena_floor.center_barrel_color == arena_floor.COLOR_RED:
                target_zone_color = arena_floor.COLOR_YELLOW
                target_zone = arena_floor.yellow_zone
                target_zone_at_center = arena_floor.yellow_zone_at_center
                obstacles = arena_floor.green_barrels_in_view
                obstacle_loc = arena_floor.green_barrel_loc
            else:
                target_zone_color = arena_floor.COLOR_BLUE
                target_zone = arena_floor.blue_zone
                target_zone_at_center = arena_floor.blue_zone_at_center
                obstacles = arena_floor.red_barrels_in_view
                obstacle_loc = arena_floor.red_barrel_loc

            # if at the zone
            if arena_floor.at_blue_zone or arena_floor.at_yellow_zone:
                self.task_state = TaskStates.UnloadBarrel
                self.barrel_unloading()
            else:
                # if the zone is at the center
                if target_zone_at_center:
                    # check if any obstacles in the path
                    if np.any(obstacle_loc == 2):
                        direction = np.random.randint(0, 2)
                        if direction == 0:
                            self.car.turn_left(self.param_turn_speed, self.param_turn_speed)
                        else:
                            self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
                        time.sleep(self.param_sleep_time)
                        self.car.move_forward(self.param_forward_speed)

                elif len(target_zone) > 0:
                    # when the zone is in the view
                    if target_zone[5] > arena_floor.img_width/2:
                        self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
                    else:
                        self.car.turn_left(self.param_turn_speed, self.param_turn_speed)

                else:  # if the zone is not in the view
                    self.car.turn_left(self.param_turn_speed, self.param_turn_speed)




    def barrel_unloading(self, arena_floor):
        pass







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

