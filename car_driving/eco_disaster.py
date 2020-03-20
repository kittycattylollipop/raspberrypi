# this is a main class for autonomous driving for eco_disaster challenge

import numpy as np
import cv2
import io
import sys
import time
from PIL import Image
import logging
import copy

import Perception as perc
import ColorMask as cmf

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)

class EcoDisaster:
    def __init__(self):
        #self.stream = io.BytesIO()  # frame buffer
        self.bgrimage = None
        self.image = None
        self.perception = perc.Perception(True, False)
        self.planning = None
        self.percOut = [None, None] #output from perception
        
        self.frame_rate = 20 
        
        self.visualize = False        

    def reset(self):
            self.percOut = [None, None] #output from perception
            self.visualize = False
            
    def run(self, frame_event, stream_lock, visualize=False, save_img=False):
        self.visualize = visualize
        counter = 0
        while True:
            try:
                # wait for the event to be set
                #logging.debug('waiting for new frame set')
                new_frame_ready = frame_event.wait()
                #logging.debug('new frame event set: %s', new_frame_ready)
                
                ################################################
                # TODO: start the auto drive task here
                ################################################                
                start = time.time()
                
                
                # read image from bugger 
                #use lock to block
                with stream_lock: 
                    #print("read stream buffer")
                    #self.stream.seek(0)
                    #jpg = self.stream.read()
                    #image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    #self.stream.truncate()
                    #print("convert bgt to rgb image")
                    self.image = self.bgrimage.copy()[..., ::-1]
                
                
                #call the perception step
                #print("call perception")
                arena_floor, color_zones = self.perception.step(self.image)
                #print("perception returned")

                with stream_lock:
                    self.percOut[0] = copy.deepcopy(arena_floor) #0 is for arena_floor
                    self.percOut[1] = copy.deepcopy(color_zones) #1 is for color_zones

                
                # visualize result and save output image
                if self.visualize:                
                    #print(arena_floor)
                    # viz results
                    imH, imW = self.bgrimage.shape[0:2]
                    # write results to image
                    whitecolor = (255, 255, 255)
                    cv2.line(self.bgrimage, (0, arena_floor.arena_ceiling), (imW - 1, arena_floor.arena_ceiling), whitecolor, 3)
                    if len(arena_floor.center_barrel) > 0:
                        cmf.draw_rect_cv2(self.bgrimage, arena_floor.center_barrel, whitecolor, 3)
                        if arena_floor.barrel_in_arm:
                            cv2.circle(self.bgrimage, tuple(arena_floor.center_barrel[5:7].astype(np.int)), 2, whitecolor, 2)
                    cmf.draw_rect_cv2(self.bgrimage, arena_floor.blue_zone, whitecolor, 3)
                    if arena_floor.blue_zone_at_center:
                        cv2.circle(self.bgrimage, tuple(arena_floor.blue_zone[5:7].astype(np.int)), 2, whitecolor, 2)
                    cmf.draw_rect_cv2(self.bgrimage, arena_floor.yellow_zone, whitecolor, 3)
                    if arena_floor.yellow_zone_at_center:
                        cv2.circle(self.bgrimage, tuple(arena_floor.yellow_zone[5:7].astype(np.int)), 2, whitecolor, 2)
                    cmf.draw_rect_cv2(self.bgrimage, arena_floor.red_barrels_in_view, (0, 0, 255), 1)
                    cmf.draw_rect_cv2(self.bgrimage, arena_floor.green_barrels_in_view, (0, 255, 0), 1)

                    cmf.draw_rect_cv2(self.bgrimage, color_zones.blackZones, (255, 0, 255), 1)
                    cmf.draw_rect_cv2(self.bgrimage, color_zones.blueZones, (255, 0, 0), 1)
                    cmf.draw_rect_cv2(self.bgrimage, color_zones.yellowZones, (0, 255, 255), 1)

                if save_img:
                    savename = '/home/pi/temp/videoimage{0:06d}.jpg'.format(counter)
                    print('eco_disaster: write to image '+ savename)
                    cv2.imwrite(savename, self.bgrimage)
                else:
                    print("EcoDis=>Frame count = %s" % counter)
                counter += 1

                # motion planning and control


                # pause and clear event for the next frame                
                cur_time = time.time()
                while cur_time - start < 1.0/self.frame_rate:  # 0.050:
                    time.sleep(0.005)
                    cur_time = time.time()
                start = cur_time                                
                frame_event.clear()
                
            except:
                e = sys.exc_info()[0]
                print("eco_disaster Exception: %s" % e)
                break

