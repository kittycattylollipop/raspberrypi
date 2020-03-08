# a class for read all sensor inputs, except camera

import time
import numpy as np
import RPi.GPIO as GPIO
from Ultrasonic import *
from servo import *


class SENSOR_TYPE:
    SENSOR_IR = "IR"
    SENSOR_US = "UltraSonic"
    SENSOR_SERVO = "Servo"
    def __init__(self):
        pass


class Sensors:
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
        self.panAngles = [90] #for servo 0
        self.tiltAngles = [90] # for servo 1

    def setServoAngles(self, pans, tilts): #input two list of angles between 0 and 180
        self.panAngles = pans
        self.tiltAngles = tilts

    def getSensorOuputs(self, read_ir=False, read_us=False):
        #get the IR inputs
        ir = []
        if read_ir:
            for i in range(3):
                ir.append(GPIO.input(self.IRs[i]))

        #get the ultrasonic distances from different angles
        dist = []
        if read_us:
            dist = np.zeros([len(self.tiltAngles), len(self.panAngles)])
            for t in range(len(self.tiltAngles)):
                # move the servo
                self.servo.setServoPwm('1', self.tiltAngles[t])
                for p in range(len(self.panAngles)):
                    #move the servo
                    self.servo.setServoPwm('0', self.panAngles[p])
                    time.sleep(0.2)
                    #get the distance
                    dist[t,p] = self.ultrasonic.get_distance()  # Get the value

        return {SENSOR_TYPE.SENSOR_IR: ir,
                SENSOR_TYPE.SENSOR_US: dist,
                SENSOR_TYPE.SENSOR_SERVO+"_tilt" : self.tiltAngles,
                SENSOR_TYPE.SENSOR_SERVO+"_pan" : self.panAngles
                }
