# this is a main class for autonomous driving for eco_disaster challenge

import numpy as np
import cv2
import io
import sys
import time
from PIL import Image
import logging

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)


class EcoDisaster:
    def __init__(self):
        self.stream = io.BytesIO() # frame buffer
        self.perception = None
        self.planning = None

    def run(self, frame_event, stream_lock):
        counter = 0
        while True:
            try:
                # wait for the event to be set
                logging.debug('waiting for new frame set')
                new_frame_ready = frame_event.wait()
                logging.debug('new frame event set: %s', new_frame_ready)
                ################################################
                # TODO: start the auto drive task here
                ################################################

                # read image from stream
                with stream_lock: #use lock to block
                    self.stream.seek(0)
                    jpg = self.stream.read()
                    image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    self.stream.truncate()

                # TEMP code write out to an image, and show image
                savename = '~/temp/videoimage' + str(counter) +'.jpg'
                cv2.imwrite(savename, image)
                counter += 1
                cv2.imshow("Capture", image)
                cv2.waitKey(1)

                # pause and clear event for the next frame
                time.sleep(0.01)
                self.event.clear()

            except:
                break

