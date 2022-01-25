import serial_comms_inator
import maneuver_inator as move
import math



'''
This is the Thrower Management module for the B T S test robot.
'''

### at thrower speed 120, it throws the ball for thrower distance
### at thrower speed ~80 it acts as ball holder



################################################################################################################################################
################################################################################################################################################

# function that takes distance to basket as input and returns thrower speed
# 800 ... 39% ... ~1,5m
# 1050 ...  51,3% ... ~2m
# 1200 ... 58,6% ... ~3m
# 1800 ... 87,9% ... ~4,5m

def throwerSpeedFromDistanceToBasket(distanceToBasket): # single input variable: distanceToBasket [m], float

    # distance to basket values range from 0 to 5m
    throwerSpeed = 0 # default to zero [unitless], int
    dutyCycle = 0 # default to zero [%], float

    ########################################################################
    maxRampRange100DC = 3.18 # derived from testing installed ramp geometry at max thrower speed, [m], float
    ########################################################################

    maxDutyCycle = 100 # constant [%], float
    maxThrowerSpeed = 2047 # constant [unitless], int

    # duty cycle = (  distance to basket * 100%  ) / tested max range with installed ramp geometry at 100% duty cycle
    dutyCycle = (distanceToBasket * maxDutyCycle) / maxRampRange100DC

    # thrower speed = (  calculated duty cycle * maximum possible thrower speed  ) / maximum possible duty cycle
    throwerSpeed = int((dutyCycle * maxThrowerSpeed) / maxDutyCycle) #using int() to ensure saved value is int
    #print(distanceToBasket*3)
    throwerSpeed = int(114,9*distanceToBasket + 237.45)
    if(throwerSpeed<330):
        throwerSpeed = 330
    elif(throwerSpeed>900):
        throwerSpeed = 900
    print(throwerSpeed)
    # return computed thrower speed integer
    return throwerSpeed # bon voyage, little green ball!

################################################################################################################################################
################################################################################################################################################

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

