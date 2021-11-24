import serial_comms_inator as serial
import math
import numpy as np


'''
This is the Maneuever Processing module for the B T S test robot.
'''


com = serial.MainboardComms()

################################################################################################################################################
################################################################################################################################################

def wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle, wheelDistanceFromCenter, robotAngularVelocity):

    wheelLinearVelocity = (robotSpeed*np.cos(np.radians(robotDirectionAngle-wheelAngle)))+(wheelDistanceFromCenter*robotAngularVelocity)

    wheelLinearVelocity = int(wheelLinearVelocity)

    return wheelLinearVelocity # mainboard units [~ m/s ~]



################################################################################################################################################
################################################################################################################################################

def calculateRelativeSpeed(deltaFactor, maxDeltaVal, minDeltaVal, maxDeltaSpeed, minAllowedSpeed, maxAllowedSpeed):
    
    absoluteDelta = abs(deltaFactor)
    
    if absoluteDelta < minDeltaVal:
        return 0
    
    deltaFrac = deltaFactor / maxDeltaVal
    absoluteDeltaFrac = abs(deltaFrac)
    
    sign = np.sign(deltaFrac)
    normalizedDeltaFrac = sign * np.power(absoluteDeltaFrac, 2)
    
    relativeSpeed = int(normalizedDeltaFrac * maxDeltaSpeed)    
    absoluteSpeed = abs(relativeSpeed)
    
    if absoluteSpeed >= minAllowedSpeed and absoluteSpeed <= maxAllowedSpeed: 
        return relativeSpeed
    
    else:
        return maxAllowedSpeed * sign 

    if relativeSpeed > maxAllowedSpeed:
        return relativeSpeed
    
    else:
        return minAllowedSpeed * sign

################################################################################################################################################
################################################################################################################################################

def allStop():
    
    allMotorSpeeds = [0,0,0,0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], 0)

################################################################################################################################################
################################################################################################################################################

# omni motion function that takes x and y robot speeds and computes robot speed and direction
def omniPlanar(robotSpeedX, robotSpeedY, robotAngularVelocity, throwerRelativeRPM, failsafe):
    
    robotSpeed = np.hypot(robotSpeedX, robotSpeedY) # hypotenuse of x and y speeds

    robotDirectionAngle = np.arctan2(robotSpeedY, robotSpeedX) # arc tangens of x and y speed is the desired travel direction

    allMotorSpeeds = [0, 0, 0, 0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    
    # physical constants wheel angle & distance from centre of robot
    wheelAngle = [120, 240, 0]
    wheelDistanceFromCenter = 0.28 # [m]

    # calculate linear velocities for each of the motors and save to allMotorSpeeds
    # MOTOR 1
    allMotorSpeeds[0] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[0], wheelDistanceFromCenter, robotAngularVelocity)

    # MOTOR 2
    allMotorSpeeds[1] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[1], wheelDistanceFromCenter, robotAngularVelocity)
    
    # MOTOR 3
    allMotorSpeeds[2] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[2], wheelDistanceFromCenter, robotAngularVelocity)

    # set thrower speed
    allMotorSpeeds[3] = int(throwerRelativeRPM)

    # send allMotorSpeeds to new class instance that writes motor speeds to mainboard to execute
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], failsafe)

################################################################################################################################################
################################################################################################################################################

# omni motion function that takes preset robot speed and direction angles
def omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe):

    allMotorSpeeds = [0, 0, 0, 0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    wheelAngle = [120, 240, 0]
    wheelDistanceFromCenter = 0.28 # [m]

    # calculate linear velocities for each of the motors and save to allMotorSpeeds
    # MOTOR 1
    allMotorSpeeds[0] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[0], wheelDistanceFromCenter, robotAngularVelocity)

    # MOTOR 2
    allMotorSpeeds[1] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[1], wheelDistanceFromCenter, robotAngularVelocity)
    
    # MOTOR 3
    allMotorSpeeds[2] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[2], wheelDistanceFromCenter, robotAngularVelocity)

    # set thrower speed
    allMotorSpeeds[3] = int(throwerRelativeRPM)
    
    # send allMotorSpeeds to new class instance that writes motor speeds to mainboard to execute
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], failsafe)


################################################################################################################################################
################################################################################################################################################
