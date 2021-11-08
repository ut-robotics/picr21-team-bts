import serial
import struct
import math
from time import sleep
from functools import partial
import time
import numpy as np
import pyrealsense2 as rs
import cv2
import motionModule as mt
mainBoardPort='/dev/ttyACM1'

filters = { #filter to find green
   "min": [20, 70, 55], # HSV minimum values
   "max": [80, 255, 160]
}

#movementMatrix read
movementMatrix = np.loadtxt('movementMatrix.txt')
movementMatrix = movementMatrix.reshape((12,16,5)); #0-2 speed, 3-4 next ball position
defaultMovement = [5,5,5]

count = 0

#camera configuration starts
pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)
device_product_line = str(device.get_info(rs.camera_info.product_line))
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#config ends

#ball find algortithm move to library
def findBall(cntMap, curVal):
    h = 12
    w = 16
    if(cntMap[curVal[0],curVal[1]] > 2):
        return curVal
    k = 1
    #TO DO replace 2 with formula
    #BFS  
    while(curVal[0]-k>=0 or curVal[1]-k>=0 or curVal[0]+k<h or curVal[1]+k<w):
        if(curVal[0]-k>=0 and (cntMap[curVal[0]-k,curVal[1]] > 2)):
            return [curVal[0]-k,curVal[1]]
        if(curVal[1]-k>=0 and (cntMap[curVal[0],curVal[1]-k] > 2)):
            return [curVal[0],curVal[1]-k]
        if(curVal[0]+k<h and (cntMap[curVal[0]+k,curVal[1]] > 2)):
            return [curVal[0]+k,curVal[1]]
        if(curVal[1]+k<w and (cntMap[curVal[0],curVal[1]+k] > 2)):
            return [curVal[0],curVal[1]+k]
        if(curVal[0]-k>=0):
            if(curVal[1]-k>=0 and (cntMap[curVal[0]-k,curVal[1]-k] > 2)):
                return [curVal[0]-k,curVal[1]-k]
            if(curVal[1]+k<w and (cntMap[curVal[0]-k,curVal[1]+k] > 2)):
                return [curVal[0]-k,curVal[1]+k]
        if((curVal[0]+k)<h):
            if(curVal[1]-k>=0 and (cntMap[curVal[0]+k,curVal[1]-k] > 2)):
                return [curVal[0]+k,curVal[1]-k]
            if(curVal[1]+k<w and (cntMap[curVal[0]+k,curVal[1]+k] > 2)):
                return [curVal[0]+k,curVal[1]-k]
        k+=1
    return [-1,-1]


pipeline.start(config)
ballPosition = [0,0]
lastRotation = -1 #coeficient for rotation
try:
    mtCntrl = mt.MotionControll(mainBoardPort)    
    while (count < 20000):
        #camera processing move to library
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        color_colormap_dim = color_image.shape		
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)		
        mask = np.array(cv2.inRange(hsv, tuple(filters["min"]), tuple(filters["max"]))) #480, 640
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', mask)
        cv2.waitKey(1)
        mask = (mask > 128).reshape(12,40,16,40).sum(axis=(1,3))//8
        #camera processing ends
        #position find for ball and relative speed set
        ballPosition = findBall(mask,ballPosition)
        if(ballPosition[0] == -1 and ballPosition[1]==-1): #no ball
            mtCntrl.setSpeeds3(defaultMovement,lastRotation*-1) #default speed
        else:
            mtCntrl.setSpeeds3(movementMatrix[ballPosition[0],ballPosition[1]],1) #set speed from matrix
            if(movementMatrix[ballPosition[0],ballPosition[1]][3] // movementMatrix[ballPosition[0],ballPosition[1]][3] != lastRotation): #1 or -1
                lastRotation = movementMatrix[ballPosition[0],ballPosition[1]][3] // movementMatrix[ballPosition[0],ballPosition[1]][3] #change rotation if is on other side
            ballPosition = [movementMatrix[ballPosition[0],ballPosition[1]][3],movementMatrix[ballPosition[0],ballPosition[1]][4]]  #set next probable ball position from matrix
        mtCntrl.send_motorspeeds()
        #position find and speed set ends
        time.sleep(0.05)
        count += 1
    pipeline.stop()
except Exception as e:
    print(e)
	
