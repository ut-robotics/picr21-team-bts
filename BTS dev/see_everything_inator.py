from enum import Enum
import cv2
import numpy as np
import pyrealsense2 as rs
import image_processor
import camera
import time

'''
This is the Image Processing module for the B T S test robot. W.I.P.!
'''

################################################################################################################################################
################################################################################################################################################

# TODO: class with all the colors
# TODO: see border boundaries

# https://github.com/Akustav/image_processing_testing/blob/master/Color.py
'''
class Color(Enum):
    OTHER = 0, np.array([0, 0, 0], dtype=np.uint8)
    GREEN = 1, np.array([0, 255, 0], dtype=np.uint8)
    MAGENTA = 2, np.array([255, 0, 255], dtype=np.uint8)
    BLUE = 3, np.array([255, 0, 0], dtype=np.uint8)
    ORANGE = 4, np.array([0, 127, 255], dtype=np.uint8)
    WHITE = 5, np.array([255, 255, 255], dtype=np.uint8)
    BLACK = 6, np.array([64, 64, 64], dtype=np.uint8)

    def __new__(cls, value, color):
        enum = object.__new__(cls)
        enum._value_ = value
        enum.color = color
        return enum

    def __int__(self):
        return self.value
'''
################################################################################################################################################
################################################################################################################################################

class frameProcessor():

    def __init__(self, setTarget): # class constructor, initialization on instance creation

        self.setTarget = setTarget # magenta or blue

        self.lowerLimitHue, self.lowerLimitSaturation, self.lowerLimitValue, self.upperLimitHue, self.upperLimitSaturation, self.upperLimitValue = readThresholdValues("ballDefault.txt")

        self.lowerLimitHueBlue, self.lowerLimitSaturationBlue, self.lowerLimitValueBlue, self.upperLimitHueBlue, self.upperLimitSaturationBlue, self.upperLimitValueBlue = readThresholdValues("blueBasket.txt")

        self.lowerLimitHueMagenta, self.lowerLimitSaturationMagenta, self.lowerLimitValueMagenta, self.upperLimitHueMagenta, self.upperLimitSaturationMagenta, self.upperLimitValueMagenta = readThresholdValues("magentaBasket.txt")

        self.kernel = np.ones((5,5),np.uint8)

        self.detector = createBlobDetector()

        self.frame = None


    def selectTarget(self, target):
        if(self.setTarget != target):
            self.setTarget = target

    def ProcessFrame(self,processor):

        keypointCount = None
        ballY = None
        ballX = None
        basketCenterX = None
        basketCenterY = None
        basketDistance = None
        '''
        # https://github.com/Akustav/image_processing_testing/blob/master/camera.py
        frames = pipeline.wait_for_frames()

        # https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1align.html
        alignedFrame = rs.align(rs.stream.color).process(frames)

        colorFrame = alignedFrame.get_color_frame()
        frame = np.asanyarray(colorFrame.get_data()) # using asanyarray from numpy to create save frame correctly every time
        depthFrame = alignedFrame.get_depth_frame()

        # https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # need to convert colorspace to hsv

        # ball threshold set up
        lowerLimits = np.array([self.lowerLimitHue, self.lowerLimitSaturation, self.lowerLimitValue])
        upperLimits = np.array([self.upperLimitHue, self.upperLimitSaturation, self.upperLimitValue])

        ballThreshold = cv2.inRange(hsv, lowerLimits, upperLimits)

        # https://stackoverflow.com/questions/42410390/blob-detection-not-working
        ballThreshold = cv2.bitwise_not(ballThreshold)

        #ballThreshold = cv2.erode(ballThreshold,self.kernel, iterations=1)

        # blue basket threshold set up
        if self.setTarget == "blue":
            basketlowerLimits = np.array([self.lowerLimitHueBlue, self.lowerLimitSaturationBlue, self.lowerLimitValueBlue])
            basketupperLimits = np.array([self.upperLimitHueBlue, self.upperLimitSaturationBlue, self.upperLimitValueBlue])

            basketWithThreshold = cv2.inRange(hsv, basketlowerLimits, basketupperLimits)
            contours, hierarchy = cv2.findContours(basketWithThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # magenta basket threshold set up
        if self.setTarget == "magenta":
            basketlowerLimits = np.array([self.lowerLimitHueMagenta, self.lowerLimitSaturationMagenta, self.lowerLimitValueMagenta])
            basketupperLimits = np.array([self.upperLimitHueMagenta, self.upperLimitSaturationMagenta, self.upperLimitValueMagenta])

            basketWithThreshold = cv2.inRange(hsv, basketlowerLimits, basketupperLimits)
            contours, hierarchy = cv2.findContours(basketWithThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key= cv2.contourArea)
            if cv2.contourArea(contour) > 200:
                cv2.drawContours(frame, contour, -1, 255, -1)
                x1, y1, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame,(x1,y1),(x1+w,y1+h),(0,255,0),3)
                basketCenterX = int(x1 + (w/2))
                basketCenterY = int(y1 + (h/2))
                #basketCenterY = int(y1)
                #print(basketCenterY)
                basketDistance = depthFrame.get_distance(basketCenterX, basketCenterY)
        

        keypoints = self.detector.detect(ballThreshold)
        keypoints = sorted(keypoints, key=lambda kp:-kp.pt[1], reverse=True)
        # add get distance from balls, select nearest ball not based on size but based on distance
        if len(keypoints) >= 1:
            ballX = keypoints[-1].pt[0]
            ballY = keypoints[-1].pt[1] 
        print(keypoints)
        
        # WIP: draw circle around closest ball to plot in debug frame
        if ballX != None:
            _, ballContours = cv2.findContours(ballThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            ballCenters = [None]*len(ballContours)
            for i, c in enumerate(ballContours):
                contours_poly[i] = cv2.approxPolyDP(c, 3, True)
                ballCenters[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
            
            for i in range(len(ballContours)):
                color = (69, 229, 33)
                cv2.drawContours(frame, contours_poly, i, color)
                cv2.circle(frame, (int(ballCenters[i][0]), int(ballCenters[i][1])), radius=int(radius[i]), color=color, thickness=2)
        
        
        keypointCount = len(keypoints)
        
        if show == True:
            #cv2.imshow("Ball Threshold", ballThreshold)
            cv2.imshow('Frame', frame)
            #cv2.imshow('Basket with Threshold', basketWithThreshold)
        '''
        #Usage of new image processing library and mapping results to existing output
        processedData = processor.process_frame()
        keypointCount = len(processedData.balls)
        if(keypointCount>0):
        #    min_distance = processedData.balls[0].distance
            ballX = processedData.balls[0].x
            ballY = processedData.balls[0].y
        #    for i in range(1,len(processedData.balls)):
        #        if(min_distance< processedData.balls[i].distance):
        #            min_distance = processedData.balls[i].distance
        #            ballX = processedData.balls[i].x
        #            ballY = processedData.balls[i].y
        if self.setTarget == "blue":
            basketCenterX = processedData.basket_b.x
            basketCenterY = processedData.basket_b.y
            if( processedData.basket_b.x != -1):
                basketDistance = processedData.basket_b.distance
        elif self.setTarget == "magenta":
            basketCenterX = processedData.basket_m.x
            basketCenterY = processedData.basket_m.y
            if( processedData.basket_m.x != -1):
                basketDistance = processedData.depth_frame[basketCenterY-1,basketCenterX-1]#basket_m.distance
                #print( processedData.depth_frame[basketCenterY-1,basketCenterX-1])
                #print(basketDistance)
        #print(np.shape(processedData.debug_frame))
        #print(processedData.debug_frame)
        return keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance, processedData.debug_frame

################################################################################################################################################
################################################################################################################################################

class RealSenseCameraManager():

    def __init__(self): # class constructor, initialization on instance call
        
        self.cameraX = 640 # camera ballX px size
        self.cameraY = 480 # camera ballY px size
        '''
        self.pipeline = rs.pipeline() # create realsense pipeline
        self.config = rs.config() # https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1config.html

        # https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1config.html#a7eac7c16b12a10f70ca93db62779ec1e
        # stream color and depth attributes
        # https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.stream.html
        # framerate = 60
        self.config.enable_stream(rs.stream.color, self.cameraX, self.cameraY, rs.format.bgr8, 60) # color camera
        self.config.enable_stream(rs.stream.depth, self.cameraX, self.cameraY, rs.format.z16, 60) # depth camera

        # https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1pipeline.html#a858f263affc80f76c5b7c9f062309ede
        self.profile = self.pipeline.start(self.config)

        # https://github.com/Akustav/image_processing_testing/blob/master/camera.py
        self.color_sensor = self.profile.get_device().query_sensors()[1]
        self.color_sensor.set_option(rs.option.enable_auto_exposure, False)
        self.color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        self.color_sensor.set_option(rs.option.white_balance, 3500)
        self.color_sensor.set_option(rs.option.exposure, 50)
        print("\nCamera successfully initialized!\n")
        '''
        cam = camera.RealsenseCamera(rgb_height=480, rgb_width=640,rgb_framerate=60)        
        self.processor = image_processor.ImageProcessor(cam, debug=True)        
        self.processor.start()
        
    def stopAllStreams(self):
        # https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1pipeline.html#a142ee1adb798f9a03f86eb90895dd8a5
        #self.pipeline.stop() # stop delivering samples
        self.processor.stop()

################################################################################################################################################
################################################################################################################################################
# another bit of study materials: https://towardsdatascience.com/image-processing-blob-detection-204dc6428dd
# does not need to be inside class def
def createBlobDetector(): # https://learnopencv.com/blob-detection-using-opencv-python-c/
    # https://www.programcreek.com/python/example/71388/cv2.SimpleBlobDetector_Params

    params = cv2.SimpleBlobDetector_Params() # https://docs.opencv.org/3.4/d0/d7a/classcv_1_1SimpleBlobDetector.html

    params.filterByArea = True
    params.minArea = 25#255
    params.maxArea = 100000000 # set max area to a large value

    params.minDistBetweenBlobs = 50

    params.filterByCircularity = False # dont filter by how circular samples are

    params.filterByInertia = False # cant filter by circularity or elipsoidicity as aspect views are not constant

    params.filterByConvexity = False # Area of the Blob / Area of it’s convex hull

    detector = cv2.SimpleBlobDetector_create(params) # create object detector with the above blob detector parameters
    return detector


################################################################################################################################################
################################################################################################################################################

# does not need to be inside class def
def readThresholdValues(filename):
    try:
        with open(filename, 'r') as reader:

            thresholdValues = reader.readline().split(",")

            if len(thresholdValues) == 6:
                lowerLimitHue = int(thresholdValues[0])
                lowerLimitSaturation = int(thresholdValues[1])
                lowerLimitValue = int(thresholdValues[2])
                upperLimitHue = int(thresholdValues[3])
                upperLimitSaturation = int(thresholdValues[4])
                upperLimitValue = int(thresholdValues[5])
                print(f"\nSuccessfully read threshold values from {filename}\n")
                return lowerLimitHue, lowerLimitSaturation, lowerLimitValue, upperLimitHue, upperLimitSaturation, upperLimitValue

            elif len(thresholdValues) != 6:
                raise ("File formatting incorrect!")
    except Exception as e:
        print(e)
        raise
    finally:
        reader.close()


################################################################################################################################################
################################################################################################################################################

