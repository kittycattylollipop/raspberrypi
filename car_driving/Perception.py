# Perception module
# the goal for this module is to analyze sensor inputs, inference the environments

import sys
import io
import time
from PIL import Image
import logging
import numpy as np
import cv2

import Sensors as ss
import ColorMask as cmf

#a data class for the world
class ArenaFloor:
    def __init__(self):
        pass

class ColorZones:
    def __init__(self):
        self.redZones = None
        self.greenZones = None
        self.blueZones = None
        self.yellowZones = None
        self.blackZones = None

class Perception:
    def __init__(self):
        self.sensors = ss.Sensors()
        self.sensors.setServoAngles([0, 90, 180], [0])

    # process for one step
    def step(self, image):
        # get the sensor inputs
        ss_out = self.sensors.sensorRead(False) #don't read all angles

        # analyze image, to get all the zone locations
        hsvImg = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        ratioImg = cmf.rgbRatioImage()
        normImg = cmf.rgbNormImage()
        colorzones = self.colorAnalysis(image, ratioImg, normImg, hsvImg)

        # inference
        self.inference(ss_out, colorzones)

    def inference(self, ss_out, color_zones):
        pass


    #get red, green barrels, get blue and yellow zones, get black walls
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

        #use HSV image
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
            rgbTh = [0, -50, -50]
            mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh)

        # connected components
        sizeTh = 50
        labCount, labels_im, connStats, connCent = cmf.maskLabeling((mask, sizeTh))

        return np.hstack((connStats, connCent))  # (stats, centroid)

    def getGreen(self, image, ratioImg, normImg, hsvImg):
        # use HSV image
        hsv_mask = cv2.inRange(hsvImg, (55, 120, 60), (70, 255, 255)).astype('bool')

        # use Ratio
        ratioTh = np.array([-0.2, 0, 3])
        rgbTh = np.array([-30, 30, -30])
        ratio_mask =  cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh)

        #use rgb norm
        if False:
            normRGBTh = np.array([-0.2, 0.6, -0.2])
            imgRGBTh = np.array([-30, 30, -30])
            norm_mask = cmf.img_mask(normImg, normRGBTh) & cmf.img_mask(image, imgRGBTh)

        mask = hsv_mask + ratio_mask

        # connected components
        sizeTh = 50
        labCount, labels_im, connStats, connCent = cmf.maskLabeling((mask, sizeTh))

        return np.hstack((connStats, connCent))  # (stats, centroid)

    def getBlue(self, image, ratioImg, normImg, hsvImg):
        # use Ratio
        ratioTh = np.array([-0.5, -0.2, -0.5])
        rgbTh = np.array([-20, -50, 40])
        mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh)

        # connected components
        sizeTh = 500
        labCount, labels_im, connStats, connCent = cmf.maskLabeling((mask, sizeTh))

        return np.hstack((connStats, connCent))  # (stats, centroid)

    def getYellow(self, image, ratioImg, normImg, hsvImg):
        # use Ratio
        ratioTh1 = np.array([-1, 3, 3])
        ratioTh2 = np.array([0.9, 0, 0])
        rgbTh = np.array([100, 100, -60])
        mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh1) & cmf.img_mask(ratioImg, ratioTh2)

        # connected components
        sizeTh = 500
        labCount, labels_im, connStats, connCent = cmf.maskLabeling((mask, sizeTh))

        return np.hstack((connStats, connCent))  # (stats, centroid)


    def getBlack(self, image, ratioImg, normImg, hsvImg):
        # use Ratio
        ratioTh = np.array([-0.6, 0, 1.5])
        rgbTh = np.array([-10, -15, -10])
        mask = cmf.img_mask(image, rgbTh) & cmf.img_mask(ratioImg, ratioTh)

        # connected components
        sizeTh = 1000
        labCount, labels_im, connStats, connCent = cmf.maskLabeling((mask, sizeTh))

        return np.hstack((connStats, connCent))  # (stats, centroid)


