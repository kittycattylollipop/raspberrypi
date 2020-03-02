import sys
from Led import *
from Motor import *
from servo import *


# Main program logic follows:
if __name__ == '__main__':
    print('Resetting Car LED, Motor and Servo ... ')
    # stop motor
    motor = Motor()
    motor.stop()
    # stop servo
    servo = Servo()
    servo.setServoPwm('0', 90)
    servo.setServoPwm('1', 90)
    # turn off led
    led = Led()
    led.colorWipe(led.strip, Color(0, 0, 0))  # turn off the light
    print("Done")
    
    

