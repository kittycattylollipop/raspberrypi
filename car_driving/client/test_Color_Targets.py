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

    perc = Perception(False)
    # read images from a directory, process and then save results
    imgnameprefix = "D:/DashanGao/Robotics/Claire/PiWarsUCSD/images/seq1/videoimage"
    for i in range(60,233):
        imgname = imgnameprefix + "{0:06d}".format(i)
        print(imgname)
        bgrimg = cv2.imread(imgname+".jpg")
        image = bgrimg[..., ::-1]

        arena_floor, color_zones = perc.step(image)

        imW, imH = image.shape[0:2]
        # write results to image

        cv2.imshow("camera image", bgrimg)
        cv2.waitKey(0)

    pass