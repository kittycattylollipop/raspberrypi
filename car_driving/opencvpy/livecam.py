import cv2
import numpy as np
#from matplotlib import pyplot as plt
#plt.rcParams['figure.figsize'] = [12, 6]

cap = cv2.VideoCapture("http://192.168.1.1:8080/?action=stream")

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    #lower_blue = np.array([50,50,50])
    #upper_blue = np.array([70,255,255])

    # Threshold the HSV image to get only blue colors
    #mask = cv2.inRange(hsv, lower_blue, upper_blue)

    im_red_mask_1 = cv2.inRange(hsv, (0, 100, 30), (15, 255, 255))
    im_red_mask_2 = cv2.inRange(hsv, (165, 100, 30), (180, 255, 255))
    #im_red_mask_1 = im_red_mask_1.astype('bool')
    #im_red_mask_2 = im_red_mask_2.astype('bool')
    #im_red_mask_hsv = im_red_mask_1 or im_red_mask_2
    im_red_mask_hsv = cv2.bitwise_or(im_red_mask_1, im_red_mask_2)
    # Bitwise-AND mask and original image
    #res = frame * np.dstack((im_red_mask_hsv, im_red_mask_hsv, im_red_mask_hsv))
    res = cv2.bitwise_and(frame, frame, mask=im_red_mask_hsv)

    cv2.imshow('frame',frame)
    cv2.imshow('mask',im_red_mask_hsv)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()