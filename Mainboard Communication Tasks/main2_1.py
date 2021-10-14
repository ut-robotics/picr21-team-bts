import serial
import struct
import math
from time import sleep
from functools import partial
import time
import numpy as np
import cv2

filters = {
   "min": [18, 131, 45], # HSV minimum values
   "max": [85, 255, 157]
}


def send_ms(ser,speeds): #unpack motorspeeds
    return send_motorspeeds(ser,speeds[0],speeds[1],speeds[2],speeds[3]) #returns motor data


def send_motorspeeds(ser,m1 = 0,m2 = 0,m3 = 0,thrower = 0): # send speeds to motors and return data
    write_this = struct.pack('<hhhHBH', m1, m2, m3, thrower, disable_failsafe, 0xAAAA)
    #print(write_this)
    ser.write(write_this)
    size = struct.calcsize('<hhhH')
    buffer_size = ser.read(size)
    #buffer_size = ser.readline()
    values = struct.unpack('<hhhH',buffer_size)
    return values #returns motor data

count = 0

disable_failsafe = 0
speed1 = 15
speed2 = 15
speed3 = 15
thrower_speed = 0

speeds = [5, 5, 5, 0]

ser = None
try:
	port='/dev/ttyACM1'
	ser = serial.Serial(port, baudrate=115200, timeout=3)
	#while True:
	print('Loop started!')
	cap = cv2.VideoCapture(4)
	while (cap.isOpened() and count < 20000):
		# 1. OpenCV gives you a BGR image
		_, bgr = cap.read()
		#cv2.imshow("bgr", bgr)

		# 2. Convert BGR to HSV where color distributions are better
		hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
		#cv2.imshow("hsv", hsv)

		# 3. Use filters on HSV image
		mask = np.array(cv2.inRange(hsv, tuple(filters["min"]), tuple(filters["max"]))) #480, 640
		print(mask.shape) #
		#cv2.imshow("mask", mask)
		mask = (mask > 128).reshape(12,40,16,40).sum(axis=(1,3))//8
		print(mask.shape)
		res = np.array(np.where(mask > 0))  
		print(res)
		if(res.size<1):
		    speeds = [5, 5, 5, 0]
		    print("No ball rotate")
		elif(7 in res[1] or 8 in res[1] or 9 in res[1]):
		    if (np.max(res[0])<11):
		        speeds = [15, 0,-15, 0]
		        print("Move forward")
		    else:
		        speeds = [5, 5, 5, 0]          
		        print("Throw")
		elif(np.max(res[1])<7 and np.max(res[0])<10):
		    speeds = [3, 3, 3, 0]
		    print("Rotate left")
		elif (np.min(res[1])>9 and np.max(res[0])<10):
		    speeds = [-3, -3, -3, 0]
		    print("Rotate right")
		else:
		    speeds = [5, 5, 5, 0] 
		motor_data = send_ms(ser,speeds)
		time.sleep(0.05)
		count += 1
	cap.release()
except Exception as e:
	print(e)
if ser != None:
	ser.close()

