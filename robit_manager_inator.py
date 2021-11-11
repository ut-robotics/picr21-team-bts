import maneuver_inator as move
import thrower_inator as throw
import time
import math
import numpy
import cv2

rcCheck = input("Manual override? [y]/n: ")

if rcCheck == 'y':
    import rc_inator

else:
    failsafe = 0
    loopCounter =0
    
    #while loopCounter <= 30:
    while True:
        robotSpeedX = 0  # [m/s]
        robotSpeedY = 0 # [m/s]
        robotAngularVelocity = 0 # [rad/s]

        # throwerRelativeRPM = 1050  # manual override variable for thrower_speed
        distanceToBasket = 1 # distance realsense returns as distance to basket [m], float
        throwerRelativeRPM = throw.speedFromDistanceToBasket(distanceToBasket) # mapped from 48 (0% duty cycle) to 2047 (100% duty cycle) [DSHOT150 protocol]
        print(throwerRelativeRPM)
        throwerRelativeRPM = 0

        move.omni_run(robotSpeedX, robotSpeedY, robotAngularVelocity, throwerRelativeRPM, failsafe) # test movement command
        time.sleep(0.1) # delay between commands being sent to mainboard 0.1s... 100ms
    
        loopCounter += 1

    move.allStop() # all stop
















'''
# Allan's suggested movement transform from image processing results:

#frameX...
#frameY...

#ball = (x;y)
#basket = (x3;y3)
#point = (x2;y2)

#speed = math.sqrt((camera_x/2-x)**2 + (camera_y-y)**2)*0.05
#direction = math.atan2(camera_x - x, camera_y - y)
#direction = math.atan2(10, 5)
'''
