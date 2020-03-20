# a class for read all sensor inputs, except camera

import time
import numpy as np
import threading

import RPi.GPIO as GPIO
from Ultrasonic import *
from servo import *

import copy

class SENSOR_TYPE:
    SENSOR_IR = "IR"
    SENSOR_US = "UltraSonic"
    SENSOR_SERVO = "Servo"
    def __init__(self):
        pass

class SensorData:
    def __init__(self):
        self.IRread = None
        self.USread = None
        self.servoPangle = None
        self.servoTangle = None

class Sensors:
    def __init__(self):
        #set up Pins for IRs
        self.IRs = [14, 15, 23]
        GPIO.setmode(GPIO.BCM)
        for i in range(3):
            GPIO.setup(self.IRs[i],GPIO.IN)

        # init the Servo (Pan,tilt to get the angle)
        self.servo = Servo()
        self.servo.setServoPwm('0', 90)
        self.servo.setServoPwm('1', 90)

        # servo angles
        self.panAngles = [90]  # for servo 0
        self.tiltAngles = [90]  # for servo 1

        # set up the ultrasonic sensor
        self.ultrasonic=Ultrasonic()
        # use a timer to measure the distance by a time interval
        self.ultrasonicTimer = threading.Timer(0.5, self.readUltrasonic)
        self.ultrasonicTimer.start()

        # data and locks
        self.data = SensorData()
        self.data_lock = threading.Lock()

    def readUltrasonic(self):
        # set servo to the home position
        self.servo.setServoPwm('0', 90)
        self.servo.setServoPwm('1', 90)
        dist = self.ultrasonic.get_distance()
        with self.data_lock:
            self.data.servoPangle = [90]
            self.data.servoTangle = [90]
            self.data.USread = [dist]

        self.ultrasonicTimer = threading.Timer(0.1, self.readUltrasonic)
        self.ultrasonicTimer.start()

    # input two list of angles between 0 and 180
    def setServoAngles(self, pans, tilts):
        self.panAngles = pans
        self.tiltAngles = tilts

    def sensorRead(self, us_full_angles=False):
        # get the IR inputs
        ir = np.zeros(3, dtype=bool)
        for i in range(3):
            ir[i] = GPIO.input(self.IRs[i])

        # get the ultrasonic distances from different angles
        dist = []
        if us_full_angles:
            # cancel timer
            self.ultrasonicTimer.cancel()
            time.sleep(0.1)
            while self.ultrasonicTimer.is_alive():
                time.sleep(0.01)
            dist = np.zeros([len(self.tiltAngles), len(self.panAngles)])
            for t in range(len(self.tiltAngles)):
                # move the servo
                self.servo.setServoPwm('1', self.tiltAngles[t])
                for p in range(len(self.panAngles)):
                    # move the servo
                    self.servo.setServoPwm('0', self.panAngles[p])
                    time.sleep(0.2)
                    # get the distance
                    dist[t,p] = self.ultrasonic.get_distance()  # Get the value

        # prepare the data to return
        self.data.IRread = ir
        if us_full_angles:
            self.data.servoPangle = self.panAngles
            self.data.servoTangle = self.tiltAngles
            self.data.USread = dist

            # restart the timer
            self.ultrasonicTimer = threading.Timer(0.2, self.readUltrasonic)
            self.ultrasonicTimer.start()

        with self.data_lock:
            data = copy.deepcopy(self.data)

        return data
