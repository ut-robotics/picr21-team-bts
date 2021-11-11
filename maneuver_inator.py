import serial_comms_inator
import math
import numpy as np

com = serial_comms_inator.Comm_Mainboard()

########################################################################

def wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle, wheelDistanceFromCenter, robotAngularVelocity):

    wheelLinearVelocity = (robotSpeed*np.cos(np.radians(robotDirectionAngle-wheelAngle)))+(wheelDistanceFromCenter*robotAngularVelocity)

    wheelLinearVelocity = int(wheelLinearVelocity)

    return wheelLinearVelocity #integer, [m/s]

########################################################################

def allStop():
    allMotorSpeeds = [0,0,0,0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], 0)

########################################################################

def forward(robotSpeed, failsafe):
    allMotorSpeeds = [robotSpeed, -robotSpeed, 0, 0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], failsafe)

def left(robotSpeed, failsafe):
    allMotorSpeeds = [-0.5*robotSpeed, -0.5*robotSpeed, robotSpeed, 0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], failsafe)


def right(robotSpeed, failsafe):
    allMotorSpeeds = [0.5*robotSpeed, 0.5*robotSpeed, -robotSpeed, 0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], failsafe)

########################################################################

def backward(robotSpeed, failsafe):
    allMotorSpeeds = [-robotSpeed, robotSpeed, 0, 0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], failsafe)

########################################################################

def omni_run(robotSpeedX, robotSpeedY, robotAngularVelocity, throwerRelativeRPM, failsafe):
    robotSpeed = np.sqrt(robotSpeedX * robotSpeedX + robotSpeedY * robotSpeedY)

    robotDirectionAngle = np.arctan2(robotSpeedY, robotSpeedX)

    allMotorSpeeds = [0, 0, 0, 0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    wheelAngle = [240, 0, 120]
    wheelDistanceFromCenter = 0.3 # [m]

    # next calculate linear velocities for each of the motors and save to allMotorSpeeds

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

########################################################################

def omni(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe):
    #robotSpeed = np.sqrt(robotSpeedX * robotSpeedX + robotSpeedY * robotSpeedY)

    #robotDirectionAngle = np.arctan2(robotSpeedY, robotSpeedX)

    allMotorSpeeds = [0, 0, 0, 0] # index 0 = motor 1, index 1 = m2, 2 = m3, 3 = thrower motor
    wheelAngle = [120, 240, 0]
    wheelDistanceFromCenter = 0.28 # [m]

    # next calculate linear velocities for each of the motors and save to allMotorSpeeds

    # MOTOR 1
    allMotorSpeeds[0] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[0], wheelDistanceFromCenter, robotAngularVelocity)

    # MOTOR 2
    allMotorSpeeds[1] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[1], wheelDistanceFromCenter, robotAngularVelocity)
    
    # MOTOR 3
    allMotorSpeeds[2] = wheelLinearVelocity(robotSpeed, robotDirectionAngle, wheelAngle[2], wheelDistanceFromCenter, robotAngularVelocity)

    # set thrower speed
    allMotorSpeeds[3] = int(throwerRelativeRPM)
    #print(allMotorSpeeds)
    # send allMotorSpeeds to new class instance that writes motor speeds to mainboard to execute
    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], failsafe)



























# function for sideaways movement
def sideways():
    #robotSpeed = sqrt(pow(sideSpeed, 2) + pow(fowardSpeed, 2))
    #dir = atan2(sideSpeed, forwardspeed)
    # side movement = (x - x3)/frameX*allMotorSpeeds
    
    #motorSpeed1 = robotSpeed * cos(dir - motorDir1)
    #motorSpeed2 = robotSpeed * cos(dir - motorDir2)
    #motorSpeed3 = robotSpeed * cos(dir - motorDir3)

    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], 0)

# function for forward movement
def forward():
    #robotSpeed = sqrt(pow(sideSpeed, 2) + pow(fowardSpeed, 2))
    #dir = atan2(sideSpeed, forwardspeed)
    # forward movement = (y-y2)/frameY*allMotorSpeeds
    
    #motorSpeed1 = robotSpeed * cos(dir - motorDir1)
    #motorSpeed2 = robotSpeed * cos(dir - motorDir2)
    #motorSpeed3 = robotSpeed * cos(dir - motorDir3)

    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], 0)

# function for stationary rotation
def rotation():
    #robotSpeed = sqrt(pow(sideSpeed, 2) + pow(fowardSpeed, 2))
    #dir = atan2(sideSpeed, forwardspeed)
    # rotation = (x-x2)/frameX*allMotorSpeeds
    #motorSpeed1 = robotSpeed * cos(dir - motorDir1)
    #motorSpeed2 = robotSpeed * cos(dir - motorDir2)
    #motorSpeed3 = robotSpeed * cos(dir - motorDir3)

    com.SendCmd2Mbd(allMotorSpeeds[0], allMotorSpeeds[1], allMotorSpeeds[2], allMotorSpeeds[3], 0)
