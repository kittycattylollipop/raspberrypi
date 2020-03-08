# Perception module
# the goal for this module is to analyze sensor inputs, inference the environments

import Sensors as ss

#a data class for the world
class ArenaFloor:
    def __init__(self):
        pass


class Perception:
    def __init__(self):
        self.sensors = ss.Sensors()
        self.sensors.setServoAngles([0, 90, 180], [0])

    # process for one step
    def step(self, image):
        # get the sensor inputs
        ss_out = self.sensors.getSensorOuputs(True, True)

        # analyze image, to get all the zone locations


    def colorAnalysis(self):
        pass

    def fusion(self):
        pass
    


