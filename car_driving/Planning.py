# motion planning and controlling module
# the goal of this module is to give the environment variables,
# send out control signals to the car

import sys
import traceback
import io
import time
import logging
import numpy as np
import enum
import threading

try:
    from Motor import Motor
    MOTOR_ON = True
except:
    MOTOR_ON = False
    print("Motor is not found")

# class for motion task states
class TaskStates(enum.Enum):
    FindBarrel = 1
    CatchBarrel = 2
    MoveBarrel = 3
    UnloadBarrel = 4


class MotionPlanning:
    def __init__(self):
        self.task_state = TaskStates.FindBarrel
        self.motor_on = MOTOR_ON
        if self.motor_on:
            self.car = Motor()
        else:
            self.car = None
        self.car_timer = None

        self.param_forward_speed = 800
        self.param_turn_speed = 1400 #1600
        self.param_speed_adjust = 0.1 #0.1
        self.param_turn_sleep_time_short = 0.5
        self.param_turn_sleep_time_long = 2.0
        self.param_backup_sleep_time = 2.0

        self.param_img_ctr_offset = 0.1

    # step function to run at each time
    def step(self, arena_floor, time_out=0):
        try:
            # cancel the time out thread
            if self.car_timer is not None:
                self.car_timer.cancel()

            # motion planning for each state
            if self.task_state == TaskStates.FindBarrel:
                print("MotionPlanning-step: task state == FindBarrel")
                self.barrel_finding(arena_floor)
            elif self.task_state == TaskStates.CatchBarrel:
                print("MotionPlanning-step: task state == CatchBarrel")
                self.barrel_catching(arena_floor)
            elif self.task_state == TaskStates.MoveBarrel:
                print("MotionPlanning-step: task state == MoveBarrel")
                self.barrel_moving(arena_floor)
            elif self.task_state == TaskStates.UnloadBarrel:
                print("MotionPlanning-step: task state == UnloadBarrel")
                self.barrel_unloading(arena_floor)

            if time_out > 0 and self.motor_on:
                self.car_timer = threading.Timer(time_out, self.car.stop)
                self.car_timer.start()
        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            if self.motor_on:
                self.car.stop()

    # find a barrel to move
    def barrel_finding(self, arena_floor):
        if len(arena_floor.red_barrels_in_view) == 0 and len(arena_floor.green_barrels_in_view) == 0:
            print("MotionPlanning-barrel_finding: turn to find barrel")
            if self.motor_on:
                self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
        elif len(arena_floor.center_barrel) > 0:  # barrel at image center, move to next state
            print("MotionPlanning-barrel_finding: found barrel, go to CatchBarrel state")
            self.task_state = TaskStates.CatchBarrel
            self.barrel_catching(arena_floor)
        else:
            min_red_dist, min_red_barrel = self.min_dist_to_center(arena_floor.red_barrels_in_view,
                                                                   arena_floor.img_width, arena_floor.img_height)
            min_green_dist, min_green_barrel = self.min_dist_to_center(arena_floor.green_barrels_in_view,
                                                                       arena_floor.img_width, arena_floor.img_height)
            if min_red_dist < min_green_dist:
                min_dist = min_red_dist
                min_barrel = min_red_barrel
            else:
                min_dist = min_green_dist
                min_barrel = min_green_barrel
            if min_barrel[5] > arena_floor.img_width / 2:
                print("MotionPlanning-barrel_finding: turn right to barrel")
                if self.motor_on:
                    self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
            else:
                print("MotionPlanning-barrel_finding: turn left to barrel")
                if self.motor_on:
                    self.car.turn_left(self.param_turn_speed, self.param_turn_speed)

    # actions for catching the barrel
    def barrel_catching(self, arena_floor):
        if arena_floor.barrel_in_arm:  # barrel is already in arm
            # move to the next state
            print("MotionPlanning-barrel_catching: barrel in arm, move to next state: MoveBarrel")
            self.task_state = TaskStates.MoveBarrel
            self.barrel_moving(arena_floor)
        else:
            if len(arena_floor.center_barrel) > 0:  # barrel at center, but not in arm yet
                # move toward the barrel, but also correcting the error
                dist_err = arena_floor.center_barrel[5] / arena_floor.img_width - 0.5
                speed_adjust_ratio = np.abs(dist_err) / self.param_img_ctr_offset * self.param_speed_adjust
                adjust_speed = int(self.param_forward_speed * (1 - speed_adjust_ratio))
                if dist_err > 0:  # the target is to the right of the center
                    # move forward with right wheels turn a bit slower (turn right slightly)
                    print("MotionPlanning-barrel_catching: move toward barrel, slight right")
                    if self.motor_on:
                        self.car.move_forward(self.param_forward_speed, adjust_speed)
                elif dist_err < 0:  # the target is to the right of the right of the center
                    # move forward with left wheels turn a bit slower (turn left slightly)
                    print("MotionPlanning-barrel_catching: move toward barrel, slight left")
                    if self.motor_on:
                        self.car.move_forward(adjust_speed, self.param_forward_speed)
                else:
                    # move forward with same speed
                    print("MotionPlanning-barrel_catching: move toward barrel")
                    if self.motor_on:
                        self.car.move_forward(self.param_forward_speed, self.param_forward_speed)
            else:  # barrel is not at the center any more
                # move back to previous state
                print("MotionPlanning-barrel_catching: Barrel not in center, go to previous sate: FindBarrel")
                self.task_state = TaskStates.FindBarrel
                self.barrel_finding(arena_floor)

    # moving barrels to the zone
    def barrel_moving(self, arena_floor):
        if not arena_floor.barrel_in_arm:
            # move back to previous state
            print("MotionPlanning-barrel_moving: barrel not in arm, move to previous state: CatchBarrel")
            self.task_state = TaskStates.CatchBarrel
            self.barrel_catching(arena_floor)
        else:
            # determine which zone to move to
            if arena_floor.center_barrel_color == arena_floor.COLOR_RED:
                print("MotionPlanning-barrel_moving: red barrel, need to move to yellow zone")
                target_zone_color = arena_floor.COLOR_YELLOW
                target_zone = arena_floor.yellow_zone
                target_zone_at_center = arena_floor.yellow_zone_at_center
                obstacles = arena_floor.green_barrels_in_view
                obstacle_loc = arena_floor.green_barrel_loc
            else:
                print("MotionPlanning-barrel_moving: green barrel, need to move to blue zone")
                target_zone_color = arena_floor.COLOR_BLUE
                target_zone = arena_floor.blue_zone
                target_zone_at_center = arena_floor.blue_zone_at_center
                obstacles = arena_floor.red_barrels_in_view
                obstacle_loc = arena_floor.red_barrel_loc

            # if at the zone
            if arena_floor.at_blue_zone or arena_floor.at_yellow_zone:
                print("MotionPlanning-barrel_moving: barrel at target zone, move to next state: UnloadBarrel")
                self.task_state = TaskStates.UnloadBarrel
                self.barrel_unloading()
            else:
                # if the zone is at the center
                if target_zone_at_center:
                    # check if any obstacles in the path
                    print("MotionPlanning-barrel_moving: target zone is at center")
                    if np.any(obstacle_loc == 2):
                        print("MotionPlanning-barrel_moving: avoid obstacle")
                        if self.motor_on:
                            direction = np.random.randint(0, 2)
                            if direction == 0:
                                self.car.turn_left(self.param_turn_speed, self.param_turn_speed)
                            else:
                                self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
                            time.sleep(self.param_turn_sleep_time_short)
                    self.car.move_forward(self.param_forward_speed, self.param_forward_speed)
                elif len(target_zone) > 0:
                    # when the zone is in the view
                    if target_zone[5] > arena_floor.img_width / 2:
                        print("MotionPlanning-barrel_moving: turn right to move target zone to center")
                        if self.motor_on:
                            self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
                    else:
                        print("MotionPlanning-barrel_moving: turn left to move target zone to center")
                        if self.motor_on:
                            self.car.turn_left(self.param_turn_speed, self.param_turn_speed)

                else:  # if the zone is not in the view
                    print("MotionPlanning-barrel_moving: looking for target zone")
                    if self.motor_on:
                        self.car.turn_left(self.param_turn_speed, self.param_turn_speed)

    # actions for unloading the barrel
    def barrel_unloading(self, arena_floor):
        # back up to unload
        # if arena_floor.at_yellow_zone or arena_floor.at_blue_zone \
        #        or arena_floor.barrel_in_arm:
        #    self.car.move_backward(self.param_forward_speed)
        # else:
        print("MotionPlanning-barrel_unloading: back up")
        if self.motor_on:
            self.car.move_backward(self.param_forward_speed)
            time.sleep(self.param_backup_sleep_time)
        # make a turn to find other barrels
        if arena_floor.yellow_zone_at_center:
            print("MotionPlanning-barrel_unloading: turn right from yellow zone")
            if self.motor_on:
                self.car.turn_right(self.param_turn_speed, self.param_turn_speed)
        else:
            print("MotionPlanning-barrel_unloading: turn left from blue zone")
            if self.motor_on:
                self.car.turn_left(self.param_turn_speed, self.param_turn_speed)
        # sleep for long turn
        time.sleep(self.param_turn_sleep_time_long)
        # change to state of finding barrel
        self.task_state = TaskStates.FindBarrel

    # find the barrel that is closest to the center of the image
    def min_dist_to_center(self, barrels, img_width, img_height):
        min_dist = img_width
        min_barrel = []
        if len(barrels) > 0:
            for barrel in barrels:
                dist_l = np.abs(barrel[0] - img_width / 2)
                dist_r = np.abs(barrel[0] + barrel[2] - img_width / 2)
                dist = min(dist_l, dist_r)
                if dist < min_dist:
                    min_dist = dist
                    min_barrel = barrel
        return min_dist, min_barrel
