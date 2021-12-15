import cv2
import json
from functools import partial
import pyrealsense2 as rs
import numpy as np

# Load saved color values from colors.json
try:
    with open("BTS_threshold_values.json", "r") as f:
        saved_colors = json.loads(f.read())
except FileNotFoundError:
    saved_colors = {}
finally:
    f.close()

print("Saved color values: ", saved_colors)
color = input("Which color would you like to threshold: ")

# Read color values from colors.json or initialize new values
if color in saved_colors:
    filters = saved_colors[color]
else:
    filters = {
        "min": [0, 0, 0], # HSV minimum values
        "max": [179, 255, 255] # HSV maximum values
    }

def save():
    saved_colors[color] = filters
    with open("colors1.json", "w") as f:
        f.write(json.dumps(saved_colors))
        f.close()
    print(f"Saved filter settings!\n{filters}\n")

def update_range(edge, channel, value):
    filters[edge][channel] = value


# Create sliders to filter colors from image
cv2.namedWindow("MASK")

# createTrackbar(name, window name, initial value, max value, function to call on change)
cv2.createTrackbar("h_min", "MASK", filters["min"][0], 179, partial(update_range, "min", 0))
cv2.createTrackbar("s_min", "MASK", filters["min"][1], 255, partial(update_range, "min", 1))
cv2.createTrackbar("v_min", "MASK", filters["min"][2], 255, partial(update_range, "min", 2))
cv2.createTrackbar("h_max", "MASK", filters["max"][0], 179, partial(update_range, "max", 0))
cv2.createTrackbar("s_max", "MASK", filters["max"][1], 255, partial(update_range, "max", 1))
cv2.createTrackbar("v_max", "MASK", filters["max"][2], 255, partial(update_range, "max", 2))


# realsense camera setup stack
cameraX = 640 
cameraY = 480   
pipeline = rs.pipeline() # create realsense pipeline
config = rs.config() # https://intelrealsense.github.io/librealsense/doxygen/classrs2_1_1config.html
config.enable_stream(rs.stream.color, cameraX, cameraY, rs.format.bgr8, 60) # color camera
config.enable_stream(rs.stream.depth, cameraX, cameraY, rs.format.z16, 60) # depth camera

profile = pipeline.start(config)

color_sensor = profile.get_device().query_sensors()[1]
color_sensor.set_option(rs.option.enable_auto_exposure, False)
color_sensor.set_option(rs.option.enable_auto_white_balance, False)
color_sensor.set_option(rs.option.white_balance, 3500)
color_sensor.set_option(rs.option.exposure, 50)

# https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.pipeline.html#pyrealsense2.pipeline.wait_for_frames
#frames = pipeline.wait_for_frames()
'''
kernel = np.ones((5,5),np.uint8)
detector = createBlobDetector()
frame = None
'''

#Open Camera frame
#cap = cv2.VideoCapture(4)

#while cap.isOpened():
while True:   
    # 1. OpenCV gives you a BGR image
    
    #_, bgr = cap.read()
    #cv2.imshow("bgr", bgr)
    frames = pipeline.wait_for_frames()
    
    alignedFrame = rs.align(rs.stream.color).process(frames)
    colorFrame = alignedFrame.get_color_frame()
    
    frame = np.asanyarray(colorFrame.get_data()) # using asanyarray from numpy to create save frame correctly every time
    
    # sanity check: what is seen?
    cv2.imshow("FRAME", frame)
    
    depthFrame = alignedFrame.get_depth_frame()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # need to convert colorspace to hsv
    # sanity check: what is seen?
    #cv2.imshow("HSV", hsv)

    # 2. Convert BGR to HSV
    #hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv", hsv)
    


    ### add robot code here? ###

    # 3. Use filters on HSV image
    mask = cv2.inRange(hsv, tuple(filters["min"]), tuple(filters["max"]))
    cv2.imshow("MASK", mask)

    key = cv2.waitKey(30)

    if key & 0xFF == ord("s"): # press s to save
        save()

    if key & 0xFF == ord("q") or key == 27: # press q or Esc to quit
        break

pipeline.stop()
cv2.destroyAllWindows()


'''
def ProcessFrame(self, pipeline, cameraX, cameraY, show):
    
    keypointCount = None
    ballY = None
    ballX = None
    basketCenterX = None
    basketCenterY = None
    basketDistance = None
    

    frames = pipeline.wait_for_frames()
    alignedFrame = rs.align(rs.stream.color).process(frames)
    colorFrame = alignedFrame.get_color_frame()
    
    frame = np.asanyarray(colorFrame.get_data()) # using asanyarray from numpy to create save frame correctly every time
    
    depthFrame = alignedFrame.get_depth_frame()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # need to convert colorspace to hsv
    
    lowerLimits = np.array([lowerLimitHueBall, lowerLimitSaturationBall, lowerLimitValueBall])
    upperLimits = np.array([upperLimitHueBall, upperLimitSaturationBall, upperLimitValueBall])
    
    ballThreshold = cv2.inRange(hsv, lowerLimits, upperLimits)
    ballThreshold = cv2.bitwise_not(ballThreshold)
    ballThreshold = cv2.erode(ballThreshold,kernel, iterations=1)
    

    if setTarget == "blue": 
        basketlowerLimits = np.array([lowerLimitHueBlue, lowerLimitSaturationBlue, lowerLimitValueBlue])
        basketupperLimits = np.array([upperLimitHueBlue, upperLimitSaturationBlue, upperLimitValueBlue])
        basketWithThreshold = cv2.inRange(hsv, basketlowerLimits, basketupperLimits)
        contours, hierarchy = cv2.findContours(basketWithThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    

    if setTarget == "magenta":
        basketlowerLimits = np.array([lowerLimitHueMagenta, lowerLimitSaturationMagenta, lowerLimitValueMagenta])
        basketupperLimits = np.array([upperLimitHueMagenta, upperLimitSaturationMagenta, upperLimitValueMagenta])
        basketWithThreshold = cv2.inRange(hsv, basketlowerLimits, basketupperLimits)
        contours, hierarchy = cv2.findContours(basketWithThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours, hierarchy = cv2.findContours(basketWithThreshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        contour = max(contours, key= cv2.contourArea)
        if cv2.contourArea(contour) > 200:
            cv2.drawContours(frame, contour, -1, 255, -1)
            x1, y1, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame,(x1,y1),(x1+w,y1+h),(0,255,0),3)
            basketCenterX = int(x1 + (w/2))
            basketCenterY = int(y1 + (h/2))
            basketDistance = depthFrame.get_distance(basketCenterX, basketCenterY)
            print(f"Distance to basket: {basketDistance}\n")
    

    if show == True:
        cv2.imshow("Ball Threshold", ballThreshold)
        cv2.imshow('Frame', frame)
        cv2.imshow('Basket with Threshold', basketWithThreshold)

    keypoints = detector.detect(ballThreshold)
    keypoints = sorted(keypoints, key=lambda kp:kp.size, reverse=True)
    

    if len(keypoints) >= 1:
        ballX = keypoints[0].pt[0]
        ballY = keypoints[0].pt[1]
    keypointCount = len(keypoints)
    
    return keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance


def createBlobDetector():
    params = cv2.SimpleBlobDetector_Params() # https://docs.opencv.org/3.4/d0/d7a/classcv_1_1SimpleBlobDetector.html
    params.filterByArea = True
    params.minArea = 255        #255
    params.maxArea = 100000000      # set max area to a large value
    params.minDistBetweenBlobs = 50
    params.filterByCircularity = False  # dont filter by how circular samples are
    params.filterByInertia = False      # cant filter by circularity or elipsoidicity as aspect views are not constant
    params.filterByConvexity = False        # Area of the Blob / Area of it’s convex hull
    detector = cv2.SimpleBlobDetector_create(params)    # create object detector with the above blob detector parameters
    return detector
'''
