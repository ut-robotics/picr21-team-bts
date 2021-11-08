import maneuver_inator as drive
import time
import math
import numpy

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

loop_counter =0

#while loop_counter <= 30:
while True:

    robotSpeed = 0  # meters per second  
    robot_Direction_Angle = 5 # radian
    robot_Angular_Velocity = 0 # radians per second

    # thrower_spd = 1050  # manual override variable for thrower_speed
    distance_to_basket = 1 # distance realsense returns as distance to basket [m], float

    thrower_spd = drive.thrower_speed_from_distance_to_basket(distance_to_basket) # mapped from 48 (0% duty cycle) to 2047 (100% duty cycle) [DSHOT150 protocol]


    drive.test_move(robotSpeed, robot_Direction_Angle, robot_Angular_Velocity, thrower_spd) # test movement command
    time.sleep(0.1) # delay between commands being sent to mainboard 0.1s... 100ms
    
    loop_counter += 1

drive.all_stop() # all stop