# a class for read all sensor inputs, except camera

import time
import numpy as np
import RPi.GPIO as GPIO
from Ultrasonic import *
from servo import *


class SENSOR_TYPE():
    SENSOR_IR = "IR"
    SENSOR_US = "UltraSonic"
    SENSOR_SERVO = "Servo"
    def __init__(self):
        pass


class Sensors():
    def __init__(self):
        #set up Pins for IRs
        self.IRs = [14, 15, 23]
        GPIO.setmode(GPIO.BCM)
        for i in range(3)
        GPIO.setup(self.IRs[i],GPIO.IN)

        #set up the ultrasonic sensor
        self.ultrasonic=Ultrasonic()
        # init the Servo (Pan,tilt to get the angle)
        self.servo = Servo()
        self.servo.setServoPwm('0', 90)
        self.servo.setServoPwm('1', 90)

        #servo angles
        self.panAngles = [] #for servo 0
        self.tiltAngles = [] # for servo 1

    def setServos(self, pans, tilts): #input two list of angles between 0 and 180
        self.panAngles = pans
        self.tiltAngles = tilts

    def getSensorOuputs(self):
        #get the IR inputs
        ir = []
        for i in range(3):
            ir.append(GPIO.input(self.IRs[i]))

        #get the ultrasonic distances
        dist = np.empty([])
        for t in self.tiltAngles:
            for p in self.panAngles:
                dist[]

        dist = ultrasonic.get_distance()  # Get the value

