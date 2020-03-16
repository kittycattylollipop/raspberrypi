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
        self.sensors = None
        self.perception = None
        self.planning = None
        self.percOut = None


    def run(self, frame_event, stream_lock):
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

                # read image from stream
                #use lock to block
                with stream_lock: 
                    # print("read stream buffer")
                    #self.stream.seek(0)
                    #jpg = self.stream.read()
                    #image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    #self.stream.truncate()
                    self.image = self.bgrimage.copy()[..., ::-1]

                #call the perception step
                arena_floor, color_zones = self.perception.step(self.image)

                with stream_lock:
                    self.percOut.arena_floor = copy.deepcopy(arena_floor)
                    self.percOut.color_zones = copy.deepcopy(color_zones)


                # TEMP code write out to an image, and show image
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

                savename = '/home/pi/temp/videoimage{0:06d}.jpg'.format(counter)
                #print('write to image '+ savename)        
                cv2.imwrite(savename, self.bgrimage)
                counter += 1

                # pause and clear event for the next frame
                #time.sleep(0.005)
                time.sleep(0.1)
                frame_event.clear()
                
            except:
                break

