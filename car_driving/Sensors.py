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
        
        self.param_servo_pan_home = 90
        self.param_servo_tilt_home = 80
        
        #set up Pins for IRs
        self.IRs = [14, 15, 23]
        GPIO.setmode(GPIO.BCM)
        for i in range(3):
            GPIO.setup(self.IRs[i],GPIO.IN)

        # init the Servo (Pan,tilt to get the angle)
        self.servo = Servo()
        self.servo.setServoPwm('0', self.param_servo_pan_home)
        self.servo.setServoPwm('1', self.param_servo_tilt_home)

        # servo angles
        self.panAngles = [self.param_servo_pan_home]  # for servo 0
        self.tiltAngles = [self.param_servo_tilt_home]  # for servo 1

        # set up the ultrasonic sensor
        self.ultrasonic=Ultrasonic()   
        self.ultrasonicTimer = None     

        # data and locks
        self.data = SensorData()
        self.data_lock = threading.Lock()
    
    def startUltrasonic(self):
        # use a timer to measure the distance by a time interval
        self.ultrasonicTimer = threading.Timer(0.01, self.readUltrasonic)
        self.ultrasonicTimer.start()
    
    def readUltrasonic(self):
        # set servo to the home position
        self.servo.setServoPwm('0', self.param_servo_pan_home)
        self.servo.setServoPwm('1', self.param_servo_tilt_home)
        dist = self.ultrasonic.get_distance()
        with self.data_lock:
            self.data.servoPangle = [self.param_servo_pan_home]
            self.data.servoTangle = [self.param_servo_tilt_home]
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
