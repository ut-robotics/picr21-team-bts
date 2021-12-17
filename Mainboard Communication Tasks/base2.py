import serial
import struct
import math
from time import sleep
from functools import partial
import time
import numpy as np
import cv2
import motionModule as mt
mainBoardPort='/dev/ttyACM1'

###camera???
filters = {
   "min": [18, 131, 45], # HSV minimum values
   "max": [85, 255, 157]
}

count = 0

try:
	mtCntrl = mt.MotionControll(mainBoardPort)
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
		mask = (mask > 128).reshape(12,40,16,40).sum(axis=(1,3))//8
		print(mask.shape)
		res = np.array(np.where(mask > 0))
		print(res)
		if(res.size<1):
		    mtCntrl.setSpeeds([5, 5, 5, 0])
		    print("No ball rotate")
		elif(7 in res[1] or 8 in res[1] or 9 in res[1]):
		    if (np.max(res[0])<11):
		        mtCntrl.setSpeeds([15, 0,-15, 0])
		        print("Move forward")
		    else:
		        mtCntrl.setSpeeds([5, 5, 5, 0])
		        print("Throw")
		elif(np.max(res[1])<7 and np.max(res[0])<10):
		    mtCntrl.setSpeeds([3, 3, 3, 0])
		    print("Rotate left")
		elif (np.min(res[1])>9 and np.max(res[0])<10):
		    mtCntrl.setSpeeds([-3, -3, -3, 0])
		    print("Rotate right")
		else:
		    mtCntrl.setSpeeds([5, 5, 5, 0])
		mtCntrl.send_motorspeeds()
		time.sleep(0.05)
		count += 1
	cap.release()
except Exception as e:
	print(e)
