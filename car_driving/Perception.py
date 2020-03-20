# Perception module
# the goal for this module is to analyze sensor inputs, inference the environments

import sys
import io
import time
from PIL import Image
import logging
import numpy as np
import cv2

import ColorMask as cmf

try:
    import Sensors as ss
except:
    print("Sensors Class is not available!")
    SENSOR_ON = False


# a data class for the world
class ArenaFloor:
    COLOR_RED = "Red"
    COLOR_GREEN = "Green"
    COLOR_BLUE = "Blue"
    COLOR_YELLOW = "Yellow"
    COLOR_BLACK = "Black"
    ZONE_DIM = 7  # left, top, width, height, area, centroid x, centroid y

    def __init__(self):
        self.img_width = 0
        self.img_height = 0
        self.arena_ceiling = 0
        self.dist_to_obj = -np.ones(4)  # front, left, back, right
        self.barrel_in_center_zone = False
        self.center_barrel_color = ""
        self.center_barrel = np.empty(0)
        self.barrel_in_arm = False
        self.blue_zone_at_center = False
        self.blue_zone = np.empty(0)
        self.yellow_zone_at_center = False
        self.yellow_zone = np.empty(0)
        self.at_blue_zone = False
        self.at_yellow_zone = False
        self.red_barrels_in_view = np.empty((0, ArenaFloor.ZONE_DIM))
        self.green_barrels_in_view = np.empty((0, ArenaFloor.ZONE_DIM))
        self.red_barrel_at_center = np.empty(0)
        # a vector representing where the barrels are
        # (0: outside of center zone, 1: in center zone, 2: at center)
        self.red_barrel_loc = np.empty(0)
        self.green_barrel_loc = np.empty(0)

    def __str__(self):
        string = "Arena Floor states: " \
                 + "\n img_width: % s " % (self.img_width) \
                 + "\n img_width: % s " % (self.img_height) \
                 + "\n  arena_ceiling: % s " % (self.arena_ceiling) \
                 + "\n  dist_to_obj: %s " % (self.dist_to_obj) \
                 + "\n  barrel_in_center_zone: %s " % (self.barrel_in_center_zone) \
                 + "\n  center_barrel_color: %s " % (self.center_barrel_color) \
                 + "\n  center_barrel: %s " % (self.center_barrel) \
                 + "\n  barrel_in_arm: %s " % (self.barrel_in_arm) \
                 + "\n  blue_zone_at_center: %s " % (self.blue_zone_at_center) \
                 + "\n  blue_zone: %s " % (self.blue_zone) \
                 + "\n  yellow_zone_at_center: %s " % (self.yellow_zone_at_center) \
                 + "\n  yellow_zone: %s " % (self.yellow_zone) \
                 + "\n  at_blue_zone: %s " % (self.at_blue_zone) \
                 + "\n  at_yellow_zone: %s " % (self.at_yellow_zone) \
                 + "\n  red_barrels_in_view: %s " % (self.red_barrels_in_view) \
                 + "\n  red_barrel_loc: %s " % (self.red_barrel_loc) \
                 + "\n  green_barrels_in_view: %s " % (self.green_barrels_in_view) \
                 + "\n  green_barrel_loc: %s " % (self.green_barrel_loc)
        return string

class ColorZones:
    def __init__(self):
        self.redZones = np.empty((0, ArenaFloor.ZONE_DIM))
        self.greenZones = np.empty((0, ArenaFloor.ZONE_DIM))
        self.blueZones = np.empty((0, ArenaFloor.ZONE_DIM))
        self.yellowZones = np.empty((0, ArenaFloor.ZONE_DIM))
        self.blackZones = np.empty((0, ArenaFloor.ZONE_DIM))


class Perception:
    def __init__(self, sensor_on=True, viz=False):
        self.sensor_on = SENSOR_ON and sensor_on
        if self.sensor_on:
            self.sensors = ss.Sensors()
            self.sensors.setServoAngles([90, 0, 180], [90])
            self.servoPositons = ['center', 'left', 'right']

        # TODO: algorithm parameters
        self.param_img_ctr_minX = 0.3
        self.param_img_ctr_maxX = 0.7
        self.param_barrel_width_th = 0.4

        self.param_ceiling_ub = 0.75

        # internal data
        self.arena_floor = ArenaFloor()
        self.color_zones = ColorZones()
        self.visualize = viz

    # process for one step
    def step(self, image):
        # print("Perception-Step: start")
        # get the sensor inputs
        if self.sensor_on:
            ss_out = self.sensors.sensorRead(False)  # don't read all angles
        else:
            ss_out = None

        # analyze image, to get all the zone locations
        hsvImg = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        ratioImg = cmf.rgbRatioImage(image)
        normImg = cmf.rgbNormImage(image)

        self.color_zones = self.colorAnalysis(image, ratioImg, normImg, hsvImg)

        # inference
        self.inference(image, ss_out, self.color_zones)

        # print("Perception-Step: end")

        return self.arena_floor, self.color_zones

    # the function to inference the world from the sensor inputs
    def inference(self, image, ss_out, color_zones):
        im_h, im_w = image.shape[0:2]
        self.arena_floor.img_width = im_w
        self.arena_floor.img_height = im_h

        # set the distance to objects
        if self.sensor_on:
            self.arena_floor.dist_to_obj[0] = ss_out.USread[0]

        # find the top of the arena by check black zone, yellow, or blue zone
        self.arena_floor.arena_ceiling = self.find_arena_ceiling(color_zones, im_h)

        # clean up all outside zones
        color_zones.blackZones = self.cleanup_zones(self.arena_floor.arena_ceiling, color_zones.blackZones)
        color_zones.yellowZones = self.cleanup_zones(self.arena_floor.arena_ceiling, color_zones.yellowZones)
        color_zones.blueZones = self.cleanup_zones(self.arena_floor.arena_ceiling, color_zones.blueZones)
        color_zones.greenZones = self.cleanup_zones(self.arena_floor.arena_ceiling, color_zones.greenZones)
        color_zones.redZones = self.cleanup_zones(self.arena_floor.arena_ceiling, color_zones.redZones)

        self.arena_floor.red_barrels_in_view = color_zones.redZones
        self.arena_floor.green_barrels_in_view = color_zones.greenZones

        # check if image center is occupied by either red or green
        red_in_zone, red_at_center, red_zones_loc = self.within_image_center(color_zones.redZones, im_w, im_h)
        self.arena_floor.red_barrel_loc = red_zones_loc
        green_in_zone, green_at_center, green_zones_loc = self.within_image_center(color_zones.greenZones, im_w, im_h)
        self.arena_floor.green_barrel_loc = green_zones_loc

        if len(red_at_center) > 0:  # red barrel at center
            self.arena_floor.barrel_in_center_zone = True
            self.arena_floor.center_barrel = red_at_center
            self.arena_floor.center_barrel_color = ArenaFloor.COLOR_RED
        else:
            if len(green_at_center) > 0:  # green target at center
                self.arena_floor.barrel_in_center_zone = True
                self.arena_floor.center_barrel = green_at_center
                self.arena_floor.center_barrel_color = ArenaFloor.COLOR_GREEN
            elif red_in_zone or green_in_zone:
                self.arena_floor.barrel_in_center_zone = True
                self.arena_floor.center_barrel = np.empty(0)
                self.arena_floor.center_barrel_color = ""
            else:
                self.arena_floor.barrel_in_center_zone = False
                self.arena_floor.center_barrel = np.empty(0)
                self.arena_floor.center_barrel_color = ""

        # check if barrel is in arm
        if len(self.arena_floor.center_barrel) > 0 and (self.arena_floor.center_barrel[2] > self.param_barrel_width_th
                                                        * im_w):
            self.arena_floor.barrel_in_arm = True
        else:
            self.arena_floor.barrel_in_arm = False

        # check if blue zone is in view and at the center
        if color_zones.blueZones.shape[0] > 0:
            left = np.min(color_zones.blueZones[:, 0])
            top = np.min(color_zones.blueZones[:, 1])
            right = np.max(color_zones.blueZones[:, 0] + color_zones.blueZones[:, 2] - 1)
            bottom = np.max(color_zones.blueZones[:, 1] + color_zones.blueZones[:, 3] - 1)
            self.arena_floor.blue_zone = np.array(
                [left, top, right - left + 1, bottom - top + 1, (right - left + 1) * (bottom - top + 1),
                 (left + right) / 2, (top + bottom) / 2])
            if (left < im_w / 2) and (right > im_w / 2):
                self.arena_floor.blue_zone_at_center = True
            else:
                self.arena_floor.blue_zone_at_center = False
        else:
            self.arena_floor.blue_zone_at_center = False
            self.arena_floor.blue_zone = np.empty(0)

        # check if at blue zone
        if self.sensor_on and self.arena_floor.blue_zone_at_center and ss_out.IRread.sum() <= 1:
            self.arena_floor.at_blue_zone = True
        else:
            self.arena_floor.at_blue_zone = False

        # check if yellow zone is in view and at the center
        if color_zones.yellowZones.shape[0] > 0:
            left = np.min(color_zones.yellowZones[:, 0])
            top = np.min(color_zones.yellowZones[:, 1])
            right = np.max(color_zones.yellowZones[:, 0] + color_zones.yellowZones[:, 2] - 1)
            bottom = np.max(color_zones.yellowZones[:, 1] + color_zones.yellowZones[:, 3] - 1)
            self.arena_floor.yellow_zone = np.array(
                [left, top, right - left + 1, bottom - top + 1, (right - left + 1) * (bottom - top + 1),
                 (left + right) / 2, (top + bottom) / 2])
            if (left < im_w / 2) and (right > im_w / 2):
                self.arena_floor.yellow_zone_at_center = True
            else:
                self.arena_floor.yellow_zone_at_center = False
        else:
            self.arena_floor.yellow_zone_at_center = False
            self.arena_floor.yellow_zone = np.empty(0)

        # check if at yellow zone
        if self.sensor_on and self.arena_floor.yellow_zone_at_center and ss_out.IRread.sum() <= 1:
            self.arena_floor.at_yellow_zone = True
        else:
            self.arena_floor.at_yellow_zone = False

    # clean up the zones outside of arena
    def cleanup_zones(self, ceiling_top, zones):
        zones_in = zones[(zones[:, 6] > ceiling_top), :]
        return zones_in

    def find_arena_ceiling(self, color_zones, im_h):
        ceiling = 0
        for zone in color_zones.blueZones:
            if (zone[1] < self.param_ceiling_ub * im_h) & (zone[1] > ceiling):
                ceiling = zone[1]
        # maxblueZone = np.argmax(color_zones.blueZones[:, 4])
        # if color_zones.blueZones[maxblueZone, 1] < self.para_ceiling_ub * im_h:
        #    ceiling = np.maximum(color_zones.blueZones[maxblueZone, 1], ceiling)
        for zone in color_zones.yellowZones:
            if (zone[1] < self.param_ceiling_ub * im_h) & (zone[1] > ceiling):
                ceiling = zone[1]
        for zone in color_zones.blackZones:
            if (zone[1] < self.param_ceiling_ub * im_h) & (zone[1] > ceiling):
                ceiling = zone[1]
        return np.int(ceiling)

    def within_image_center(self, zones, imW, imH):
        in_zone = False
        left = self.param_img_ctr_minX * imW
        right = self.param_img_ctr_maxX * imW

        center_zone = np.empty(0)
        zones_loc = np.empty(0)
        if zones.size == 0:
            return in_zone, center_zone, zones_loc
        # get zone index having the largest overlap with the center zone
        overlap = np.minimum(zones[:, 0] + zones[:, 2] - 1, right) - np.maximum(zones[:, 0], left)
        idx = np.argmax(overlap)
        if overlap[idx] > 0:
            in_zone = True
            z = zones[idx, :]
            if (z[0] < imW / 2) and (z[0] + z[2] > imW / 2):
                center_zone = z

        loc1 = (overlap > 0).astype(np.int)
        loc2 = np.logical_and((zones[:, 0] < imW / 2), (zones[:, 0] + zones[:, 2] > imW / 2))
        loc2 = loc2.astype(np.int)
        zones_loc = loc1 + loc2
        return in_zone, center_zone, zones_loc

    # get red, green barrels, get blue and yellow zones, get black walls
    def colorAnalysis(self, image, ratioImg, normImg, hsvImg):
        clrzone = ColorZones()
        clrzone.redZones = self.getRed(image, ratioImg, normImg, hsvImg)
        clrzone.greenZones = self.getGreen(image, ratioImg, normImg, hsvImg)
        clrzone.yellowZones = self.getYellow(image, ratioImg, normImg, hsvImg)
        clrzone.blueZones = self.getBlue(image, ratioImg, normImg, hsvImg)
        clrzone.blackZones = self.getBlack(image, ratioImg, normImg, hsvImg)
        return clrzone

    def getRed(self, image, ratioImg, normImg, hsvImg):
        useHSV = False
        useRatio = True

        # use HSV image
        mask = []
        if useHSV:
            im_red_mask_1 = cv2.inRange(hsvImg, (0, 100, 30), (15, 255, 255))
            im_red_mask_2 = cv2.inRange(hsvImg, (165, 100, 30), (180, 255, 255))
            im_red_mask_1 = im_red_mask_1.astype('bool')
            im_red_mask_2 = im_red_mask_2.astype('bool')
            mask = im_red_mask_1 + im_red_mask_2

        # use Ratio
        if useRatio:
            ratioTh = [2, 2, 0]
            rgbTh = [50, -50, -50]
            mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh)

        # connected components
        sizeTh = 80
        labCount, labels_im, connStats, connCent = cmf.maskLabeling(mask, sizeTh)

        # visualize
        if self.visualize:
            cmf.imshow_components(labels_im, "red")

        return np.hstack((connStats, connCent))  # (stats, centroid)

    def getGreen(self, image, ratioImg, normImg, hsvImg):
        # use HSV image
        # hsv_mask = cv2.inRange(hsvImg, (55, 120, 60), (70, 255, 255)).astype('bool')
        hsv_mask = cv2.inRange(hsvImg, (55, 120, 60), (75, 255, 255)).astype('bool')

        # use Ratio (for dark green)
        ratioTh = np.array([-0.2, 0, 3])
        rgbTh = np.array([-20, 50, -20])
        ratio_mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh)

        # use rgb norm
        if False:
            normRGBTh = np.array([-0.2, 0.6, -0.2])
            imgRGBTh = np.array([-30, 30, -30])
            norm_mask = cmf.img_mask(normImg, normRGBTh) & cmf.img_mask(image, imgRGBTh)

        mask = hsv_mask + ratio_mask
        if self.visualize:
            cmf.visMask_cv(hsv_mask.astype('uint8') * 255, 'green-hsv')
            cmf.visMask_cv(ratio_mask.astype('uint8') * 255, 'green-ratio')
            cmf.visMask_cv(mask.astype('uint8') * 255, 'green-mask')

        # connected components
        sizeTh = 120
        labCount, labels_im, connStats, connCent = cmf.maskLabeling(mask, sizeTh)

        # visualize
        if self.visualize:
            cmf.imshow_components(labels_im, "green")

        return np.hstack((connStats, connCent))  # (stats, centroid)

    def getBlue(self, image, ratioImg, normImg, hsvImg):

        # use HSV image
        # hsv_mask = cv2.inRange(hsvImg, (100, 50, 50), (120, 255, 255)).astype('bool')
        hsv_mask = cv2.inRange(hsvImg, (100, 50, 50), (130, 255, 255)).astype('bool')

        # use Ratio
        ratioTh = np.array([-0.5, -0.2, -0.6])
        rgbTh = np.array([-20, -50, 40])
        ratio_mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh)

        mask = hsv_mask + ratio_mask
        # if self.visualize:
        #   cmf.visMask_cv(hsv_mask.astype('uint8')*255,'blue-hsv')
        #   cmf.visMask_cv(ratio_mask.astype('uint8')*255,'blue-ratio')
        #   cmf.visMask_cv(mask.astype('uint8') * 255, 'blue-mask')

        # connected components
        sizeTh = 800
        labCount, labels_im, connStats, connCent = cmf.maskLabeling(mask, sizeTh)

        # visualize
        if self.visualize:
            cmf.imshow_components(labels_im, "blue")

        return np.hstack((connStats, connCent))  # (stats, centroid)

    def getYellow(self, image, ratioImg, normImg, hsvImg):
        # use Ratio
        ratioTh1 = np.array([-1, 3, 3])
        ratioTh2 = np.array([0.85, 0, 0])
        rgbTh = np.array([100, 100, -60])
        mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh1) & cmf.img_mask(ratioImg, ratioTh2)
        # cmf.visMask(image, mask)
        # connected components
        sizeTh = 800
        labCount, labels_im, connStats, connCent = cmf.maskLabeling(mask, sizeTh)

        # visualize
        if self.visualize:
            cmf.imshow_components(labels_im, "yellow")

        return np.hstack((connStats, connCent))  # (stats, centroid)

    def getBlack(self, image, ratioImg, normImg, hsvImg):
        # use Ratio
        # ratioTh = np.array([-0.6, 0, 1.5])
        # rgbTh = np.array([-10, -15, -10])
        ratioTh = np.array([-0.6, 0, 1.5])
        rgbTh = np.array([-10, -15, -10])
        mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh)

        # connected components
        sizeTh = 1000
        labCount, labels_im, connStats, connCent = cmf.maskLabeling(mask, sizeTh)

        # visualize
        if self.visualize:
            cmf.imshow_components(labels_im, "black")

        return np.hstack((connStats, connCent))  # (stats, centroid)
