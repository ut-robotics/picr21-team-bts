import serial_comms_inator
import math

com = serial_comms_inator.Comm_Mainboard()


# function for sideaways movement
def sideways():
    #robotSpeed = sqrt(pow(sideSpeed, 2) + pow(fowardSpeed, 2))
    #dir = atan2(sideSpeed, forwardspeed)
    # side movement = (x - x3)/frameX*all_motor_speeds
    
    #motorSpeed1 = robotSpeed * cos(dir - motorDir1)
    #motorSpeed2 = robotSpeed * cos(dir - motorDir2)
    #motorSpeed3 = robotSpeed * cos(dir - motorDir3)

    com.SendCmd2Mbd(all_motor_speeds[0], all_motor_speeds[1], all_motor_speeds[2], all_motor_speeds[3], 0)

# function for forward movement
def forward():
    #robotSpeed = sqrt(pow(sideSpeed, 2) + pow(fowardSpeed, 2))
    #dir = atan2(sideSpeed, forwardspeed)
    # forward movement = (y-y2)/frameY*all_motor_speeds
    
    #motorSpeed1 = robotSpeed * cos(dir - motorDir1)
    #motorSpeed2 = robotSpeed * cos(dir - motorDir2)
    #motorSpeed3 = robotSpeed * cos(dir - motorDir3)

    com.SendCmd2Mbd(all_motor_speeds[0], all_motor_speeds[1], all_motor_speeds[2], all_motor_speeds[3], 0)

# function for stationary rotation
def rotation():
    #robotSpeed = sqrt(pow(sideSpeed, 2) + pow(fowardSpeed, 2))
    #dir = atan2(sideSpeed, forwardspeed)
    # rotation = (x-x2)/frameX*all_motor_speeds
    #motorSpeed1 = robotSpeed * cos(dir - motorDir1)
    #motorSpeed2 = robotSpeed * cos(dir - motorDir2)
    #motorSpeed3 = robotSpeed * cos(dir - motorDir3)

    com.SendCmd2Mbd(all_motor_speeds[0], all_motor_speeds[1], all_motor_speeds[2], all_motor_speeds[3], 0)

########################################################################
# function that takes desired robotSpeed, direction angle and motor angular offset to return its linear velocity
def wheelLinearVelocity(robotSpeed, robot_Direction_Angle, motor_angle): # motor_angle is a constant of each wheel

    wheel_lin_velocity = int( robotSpeed * math.cos( robot_Direction_Angle - math.radians( 60 + motor_angle ))) # 60 = wheel distance from centre

    return wheel_lin_velocity #integer, meters per second

########################################################################

def all_stop():
    all_motor_speeds = [0,0,0,0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    com.SendCmd2Mbd(all_motor_speeds[0], all_motor_speeds[1], all_motor_speeds[2], all_motor_speeds[3], 0)

########################################################################

def test_move(robotSpeed, robot_Direction_Angle, robot_Angular_Velocity, thrower_relative_rpm):
    # need to define motor speeds list then compute each motor's linear velocity
    all_motor_speeds = [0,0,0,0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor

    # next calculate linear velocities for each of the motors and save to all_motor_speeds

    # MOTOR 1
    all_motor_speeds[0] = robot_Angular_Velocity + wheelLinearVelocity(robotSpeed, robot_Direction_Angle, 120)

    # MOTOR 2
    all_motor_speeds[1] = robot_Angular_Velocity + wheelLinearVelocity(robotSpeed, robot_Direction_Angle, 240)
    
    # MOTOR 3
    all_motor_speeds[2] = robot_Angular_Velocity + wheelLinearVelocity(robotSpeed, robot_Direction_Angle, 0)

    # set thrower speed
    all_motor_speeds[3] = int(thrower_relative_rpm)

    # send all_motor_speeds to new class instance that writes motor speeds to mainboard to execute
    com.SendCmd2Mbd(all_motor_speeds[0], all_motor_speeds[1], all_motor_speeds[2], all_motor_speeds[3], 0)

########################################################################

# function that takes distance to basket as input and returns thrower speed
# 800 ... 39% ... ~1,5m
# 1050 ...  51,3% ... ~2m
# 1200 ... 58,6% ... ~3m
# 1800 ... 87,9% ... ~4,5m
def thrower_speed_from_distance_to_basket(distance_to_basket): # single input variable: distance_to_basket [m], float
    
    # distance to basket values range from 0 to 5m
    thrower_speed = 0 # default to zero [unitless], int
    duty_cycle = 0 # default to zero [%], float
    
    ########################################################################
    max_ramp_range_100DC = 5 # derived from testing installed ramp geometry at max thrower speed, [m], float
    ########################################################################

    max_duty_cycle = 100 # constant [%], float
    max_thrower_speed = 2047 # constant [unitless], int

    # duty cycle = (  distance to basket * 100%  ) / tested max range with installed ramp geometry at 100% duty cycle
    duty_cycle = (distance_to_basket*max_duty_cycle)/max_ramp_range_100DC

    # thrower speed = (  calculated duty cycle * maximum possible thrower speed  ) / maximum possible duty cycle
    thrower_speed = int((duty_cycle*max_thrower_speed)/max_duty_cycle) #using int() to ensure saved value is int

    # return computed thrower speed integer
    return thrower_speed # bon voyage, little green ball!



'''
Allan: 
A single unified function can be done but it will have a ton of arguments.
Overall the rate of acceleration per state differs also, driving to the ball can be done rapidly, with little care to precision, while aiming should be done with care and acceleration can not be as rough
If I had to make a function, I would do something like this:
calculateMotion(targetOffset, maxTargetOffset, minTargetOffset, minSpeed, maxSpeed, speedScaleFunction)
targetOffset = error between wanted object location and actual location
maxTargetOffset = maximum error expected, for centralizing a ball, for example, half of the screen width
minTargetOffset = offset that is considered to round down to 0. Useful for when speed is more important that accuracy, for example throwing from close range (centering of basket does not need to be spot on). Also can be used to reduce some noise effects when target is close to desired area
minSpeed = minimum turning speed that is used while offset is not minimized. Needed because robot tends to not move on ultra small speeds.
maxSpeed = maximum allowed speed.
speedScaleFunction = mapping function that describes how the normalized error will be scaled. Useful when you want to accelerate much with distant objects but need slow precise movements close-by in the same state.
----
you can also add previousSpeed and acceleration parameters to controll the rate of change in speeds
For simplicity I would skip the speedScaleFunction for now
'''