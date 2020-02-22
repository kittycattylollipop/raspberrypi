import time
import curses 
from Motor import *
import RPi.GPIO as GPIO
IR01 = 14
IR02 = 15
IR03 = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR01,GPIO.IN)
GPIO.setup(IR02,GPIO.IN)
GPIO.setup(IR03,GPIO.IN)

def input_char(message):
    try:
        win = curses.initscr()
        win.addstr(0, 0, message)
        while True: 
            ch = win.getch()
            if ch in range(32, 127): 
                break
            time.sleep(0.05)
    finally:
        curses.endwin()
    return chr(ch)

class Line_Following:
    def __init__(self):
        self.forward_speed = 800
        self.backward_speed = 800
        self.turn_slow_wheel = 1500
        self.turn_fast_wheel = 2500
        self.sharp_slow_wheel = 2000
        self.sharp_fast_wheel = 4000
		
    def auto_drive(self):
        print('auto drive starts...')
        while True:
			ir1=GPIO.input(IR01)
			ir2=GPIO.input(IR02)
			ir3=GPIO.input(IR03)
			
			print(ir1, ir2, ir3)
			if ir1==0 and ir2==1 and ir3==0:
				PWM.move_forward(self.forward_speed)
			elif ir1==1 and ir2==0 and ir3==0:
				PWM.turn_left(self.turn_slow_wheel, self.turn_fast_wheel)
			elif ir1==0 and ir2==0 and ir3==1:
				PWM.turn_right(self.turn_fast_wheel, self.turn_slow_wheel)
			elif ir1==1 and ir2==1 and ir3==0:
				PWM.turn_left(self.sharp_slow_wheel, self.sharp_fast_wheel)
			elif ir1==0 and ir2==1 and ir3==1:
				PWM.turn_right(self.sharp_fast_wheel, self.sharp_slow_wheel)
			elif ir1==1 and ir2==1 and ir3==1:
				#pass
				PWM.stop()
				break
			elif ir1==0 and ir2==0 and ir3==0:
				pass
				#PWM.stop()
				#break
			
			time.sleep(0.01)
        
        print('auto drive ended...')
		
    def remote_drive(self):        
        print('remote drive starts...')
        while True:
            c = input_char('')
            if c.lower() == 'w':
                PWM.move_forward(self.forward_speed)
            elif c.lower() == 's':
                 PWM.move_backward(self.backward_speed)
            elif c.lower() == 'a':
                PWM.turn_left(self.turn_slow_wheel, self.turn_fast_wheel)
            elif c.lower() == 'd':
                PWM.turn_right(self.turn_fast_wheel, self.turn_slow_wheel)
            elif c.lower() == ' ':
                PWM.stop()      
            elif c == 'E':
				PWM.stop()
				break
            elif c=='[': #arrows are two chars: "[A": "up", "[B": "down", "[C": "right", "[D": "left"
                c = input_char('')
                if c=='A':
                    PWM.move_forward(self.forward_speed)
                elif c == 'B':
                    PWM.move_backward(self.backward_speed)
                elif c == 'D':
                    PWM.turn_left(self.turn_slow_wheel, self.turn_fast_wheel)
                elif c  == 'C':
                    PWM.turn_right(self.turn_fast_wheel, self.turn_slow_wheel)
                    
            #time.sleep(0.02)
            #PWM.stop()
        print('remote drive ended...')
    
infrared=Line_Following()

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting right now... ')
    try:
		choice=input('Please choose 1: Manual Control, or 2: Auto Drive     ')
		if choice==1:
			infrared.remote_drive()  
		elif choice==2:     
			infrared.auto_drive()
        
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        PWM.stop()
	






