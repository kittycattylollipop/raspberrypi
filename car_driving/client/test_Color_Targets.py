#  code to debug the perception code

import sys
import cv2
import numpy as np
from matplotlib import pyplot as plt
plt.rcParams['figure.figsize'] = [12, 6]

from matplotlib import patches as patches
import importlib

from Perception import *


# Main program logic follows:
if __name__ == '__main__':

    perc = Perception(True, True)
    # read images from a directory, process and then save results
    imgnameprefix = "D:/DashanGao/Robotics/Claire/PiWarsUCSD/images/seq1/videoimage"
    for i in range(60, 233):  # (60,233):  79, 109, 209
        imgname = imgnameprefix + "{0:06d}".format(i)
        print(imgname)
        bgrimg = cv2.imread(imgname+".jpg")
        image = bgrimg[..., ::-1]

        arena_floor, color_zones = perc.step(image)

        # display information
        print(arena_floor)

        imH, imW = image.shape[0:2]
        # write results to image
        whitecolor = (255, 255, 255)
        cv2.line(bgrimg, (0, arena_floor.arena_ceiling), (imW-1, arena_floor.arena_ceiling),whitecolor, 3)
        if len(arena_floor.center_barrel)>0:
            cmf.draw_rect_cv2(bgrimg, arena_floor.center_barrel, whitecolor,3)
            if arena_floor.barrel_in_arm:
                cv2.circle(bgrimg, tuple(arena_floor.center_barrel[5:7].astype(np.int)),2,whitecolor,2)
        cmf.draw_rect_cv2(bgrimg, arena_floor.blue_zone, whitecolor, 3)
        if arena_floor.blue_zone_at_center:
            cv2.circle(bgrimg, tuple(arena_floor.blue_zone[5:7].astype(np.int)), 2, whitecolor, 2)
        cmf.draw_rect_cv2(bgrimg, arena_floor.yellow_zone, whitecolor, 3)
        if arena_floor.yellow_zone_at_center:
            cv2.circle(bgrimg, tuple(arena_floor.yellow_zone[5:7].astype(np.int)), 2, whitecolor, 2)
        cmf.draw_rect_cv2(bgrimg, arena_floor.red_barrels_in_view, (0,0,255), 1)
        cmf.draw_rect_cv2(bgrimg, arena_floor.green_barrels_in_view, (0,255,0),1)

        cmf.draw_rect_cv2(bgrimg, color_zones.blackZones, (255,0,255), 1)
        cmf.draw_rect_cv2(bgrimg, color_zones.blueZones, (255,0,0), 1)
        cmf.draw_rect_cv2(bgrimg, color_zones.yellowZones, (0,255,255), 1)
        cv2.imshow("camera image", bgrimg)
        cv2.waitKey(0)

    pass