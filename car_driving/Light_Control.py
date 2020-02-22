#!/usr/bin/sudo python

import time
from Get_Input import *
from Led import *
from random import randint
from random import seed
import numpy as np

seed(time.time())

led=Led()

led.ORDER = "RGB"

R=[0]*9
G=[0]*9
B=[0 for x in range(9)]
ledindex = 0
led_num = 0
step=1

try:
	while True:
		c = input_char('')
		if c.lower() in ['1','2','3','4','5','6','7','8']:
			led_num = int(c.lower())
			ledindex = 1 << (led_num-1)
		elif c.lower() == '0':
			if led_num == 9:
				led_num = 0
			R[0]=R[led_num]
			G[0]=G[led_num]
			B[0]=B[led_num]
			led_num=0
			ledindex = 0
		elif c.lower() == '9':
			if ledindex != 0:
				R[led_num] = randint(0,255)
				G[led_num] = randint(0,255)
				B[led_num] = randint(0,255)
			else:
				for i in range(0,9):
					R[i] = randint(0,255)
					G[i] = randint(0,255)
					B[i] = randint(0,255)
				led_num = 9			
		elif c.lower() == 'w':
			if led_num == 9:
				R = list(np.minimum(np.array(R)+step,255))
			else:
				R[led_num]=min(R[led_num]+step,255)
		elif c.lower() == 's':
			if led_num == 9:
				R = list(np.maximum(np.array(R)-step,0))
			else:
				R[led_num]=max(R[led_num]-step,0)
		elif c.lower() == 'e':
			if led_num == 9:
				G = list(np.minimum(np.array(G)+step,255))
			else:
				G[led_num]=min(G[led_num]+step,255)
		elif c.lower() == 'd':
			if led_num == 9:
				G = list(np.maximum(np.array(G)-step,0))
			else:
				G[led_num]=max(G[led_num]-step,0)
		elif c.lower() == 'r':
			if led_num == 9:
				B = list(np.minimum(np.array(B)+step,255))
			else:
				B[led_num]=min(B[led_num]+step,255)
		elif c.lower() == 'f':
			if led_num == 9:
				B = list(np.maximum(np.array(B)-step,0))
			else:
				B[led_num]=max(B[led_num]-step,0)	
		elif c.lower() == ' ':
			if led_num == 9:
				led_num=0
			R[led_num]=0
			G[led_num]=0
			B[led_num]=0
		
		#print(R,G,B)
		
		if led_num == 9:
			for i in range(0,8):
				led.ledIndex(1 << i,R[i],G[i],B[i])
		elif led_num==0:
			led.colorWipe(led.strip, Color(R[led_num],G[led_num],B[led_num]),0)	
		else:
			led.ledIndex(ledindex,R[led_num],G[led_num],B[led_num]) 

	led.colorWipe(led.strip, Color(0,0,0),0)
except KeyboardInterrupt:
	led.colorWipe(led.strip, Color(0,0,0),0)
