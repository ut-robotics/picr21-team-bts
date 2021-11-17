from enum import Enum
import time
import cv2
import numpy
import maneuver_inator as move
import thrower_inator as throw
import see_everything_inator as eyes
import rc_inator as gui



'''
This is a mirror of the Main Game Logic module for the B T S test robot. W.I.P.!
premise of this module is to offload the actual game logic processing to here to
achieve better code aesthetic and clarity and reduce cognitive load
when developing game logic functions.
'''


################################################################################################################################################
################################################################################################################################################

# enum class with all key state definitions 
class State(Enum):
    IDLE = 0 # idling state
    FIND = 1 # finding ball state ( rotate until ball(s) found )
    DRIVE = 2 # drive to found ball
    HOLD = 3 # not implemented but after ball certain distance from bot, collect ball
    AIM = 4 # orbit/rotate until aligned with basket centre
    THROW = 5 # execute throw (if hold used, may need 2 throws)

stateSwitch = {
    State.IDLE: Idle,
    State.FIND: Find,
    State.DRIVE: Drive,
    State.HOLD: Hold,
    State.AIM: Aim,
    State.THROW: Throw
}

eyeCam = eyes.RealSenseCameraManager() # fully preconfigures camera

currentState = State.IDLE # default starting state

setTarget = "magenta" #set target basket "blue" or "magenta" (default)

sight = eyes.frameProcessor(setTarget) # create image core processing instance


################################################################################################################################################
################################################################################################################################################

def Hold(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    '''
    potentially can implement a hold ball feature
    idea: find... ball seen ==> drive... drive to ball
    ==> hold... collect ball (hold) 
    ==> aim... rotate to align with basket target
    ==> throw... throw short then execute full throw...

    ?? is this faster than:
    find... ball seen ==> drive... drive to ball
    ==> aim... orbit ball until aligned with basket
    ==> throw... throw ball to basket

    depends on how accurate the orbiting is
    depends on how precise the aiming is
    depends on how fast the robot can move
    depends on whether or not the mini throw before the main throw is even possible...
    having to execute aiming multiple times may improve overall results
    '''
    return State.AIM

################################################################################################################################################
################################################################################################################################################

def Drive(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    
    if keypointCount > 0:
        
        # relative motion straight towards the ball

        deltaFactorX = ballX - eyeCam.cameraX/2
        deltaFactorY = 390 - ballY
        
        maxDeltaValY = eyeCam.cameraY
        maxDeltaValX = eyeCam.cameraX
        
        minDeltaVal = 5        
        
        minAllowedSpeedForward = 5
        minAllowedSpeedRotation = 2
        
        maxAllowedSpeedForward = 40
        maxAllowedSpeedRotation = 20
        
        maxDeltaSpeedForward = 350
        maxDeltaSpeedRotation = 150
        
        throwerRelativeRPM = 0
        failsafe = 0

        forwardSpeed = move.calculateRelativeSpeed(deltaFactorY, maxDeltaValY, minDeltaVal, maxDeltaSpeedForward, minAllowedSpeedForward, maxAllowedSpeedForward)
        sideSpeed = 0 # robotSpeedX, robotSpeedY, robotAngularVelocity
        rotationSpeed = move.calculateRelativeSpeed(deltaFactorX, maxDeltaValX, minDeltaVal, maxDeltaSpeedRotation, minAllowedSpeedRotation, maxAllowedSpeedRotation)
                
        move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
        
        if 450 > ballY > 300 : # distance to ball condition for aim to start
    
            return State.AIM
    
    if keypointCount <= 0 or None:
    
        return State.FIND

    return State.DRIVE

################################################################################################################################################
################################################################################################################################################

def Find(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    
    # rotate left around robot central axis

    robotSpeed = 0
    robotDirectionAngle = 0
    robotAngularVelocity = 60
    throwerRelativeRPM = 0
    failsafe = 0 

    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
    
    # if any balls found, set state DRIVE

    if keypointCount >= 1:
        Drive(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
        return State.DRIVE
    
    # or set state FIND
    return State.FIND

################################################################################################################################################
################################################################################################################################################

def Aim(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    
    basketInFrame = basketCenterX is not None

    if ballX is None :
        return State.FIND

    if not basketInFrame:
        deltaFactorX = eyeCam.cameraX
    
    else:
        deltaFactorX = ballX - basketCenterX

    deltaRotationFactorX = ballX - eyeCam.cameraX/2
    
    deltaFactorY = 450 - ballY
    
    minDeltaVal = 7
    minAllowedSpeed = 3
    maxDeltaSpeed = 150
    maxAllowedSpeed = 30
    throwerRelativeRPM = 0
    failsafe = 0
    
    forwardSpeed = move.calculateRelativeSpeed(deltaFactorY, eyeCam.cameraY, minDeltaVal, minAllowedSpeed, maxDeltaSpeed, maxAllowedSpeed)
    sideSpeed = move.calculateRelativeSpeed(deltaFactorX, eyeCam.cameraX, minDeltaVal, minAllowedSpeed, maxDeltaSpeed, maxAllowedSpeed)
    rotationSpeed = move.calculateRelativeSpeed(deltaRotationFactorX, eyeCam.cameraX, minDeltaVal, minAllowedSpeed-1, maxDeltaSpeed, maxAllowedSpeed)
        
    move.omniPlanar(-sideSpeed, forwardSpeed, -rotationSpeed, throwerRelativeRPM, failsafe)
    
    # if ball is close to the robot and if the basket is centered to eyeCam x view, stop then throw the ball at basket 
    
    if basketInFrame and 315 <= basketCenterX <= 325 and ballY >= 410: 
       
       move.allStop()
       
       return State.THROW

    return State.AIM

################################################################################################################################################
################################################################################################################################################

throwIterand = 0

def Throw(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    
    global throwIterand
    
    if throwIterand >= 10:
        
        throwIterand = 0
        
        return State.FIND
    
    if keypointCount >= 1:
        
        basketInFrame = basketCenterX is not None

        if not basketInFrame:
            deltaFactorX = eyeCam.cameraX
        
        else:
            deltaFactorX = ballX - basketCenterX
        
        deltaRotationFactorX = ballX - eyeCam.cameraX/2
        
        deltaFactorY = 500 - ballY
        
        minSpeed = 10
        minAllowedSpeed = 10

        maxSpeed = 30
        maxAllowedSpeed = 30

        minDelta = 6
        minDeltaVal = 6
        
        maxDeltaSpeed = 150

        throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(basketDistance)
        forwardSpeed = move.calculateRelativeSpeed(deltaFactorY, eyeCam.cameraY, minDeltaVal, maxDeltaSpeed, minAllowedSpeed, maxAllowedSpeed)
        #sideSpeed = move.calculateRelativeSpeed(deltaFactorX, eyeCam.cameraX, minDeltaVal, maxDeltaSpeed, minAllowedSpeed, maxAllowedSpeed)
        #rotationSpeed = move.calculateRelativeSpeed(rot_delta_x, eyeCam.cameraX, minDeltaVal, 100, 3, maxAllowedSpeed)
        sideSpeed = 0
        rotationSpeed = 0

        delta, maxDelta, minDelta, minSpeed, maxDeltaSpeed, maxSpeed
        deltaFactor, maxDeltaVal, minDeltaVal, maxDeltaSpeed, minAllowedSpeed, maxAllowedSpeed

        move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
    
    if keypointCount <= 0:

        throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(basketDistance)
        
        move.omniPlanar(0, 15, 0, throwerRelativeRPM)
        
        throwIterand += 1
    
    return State.THROW

################################################################################################################################################
################################################################################################################################################

def Idle(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    
    move.allStop()
    
    return State.IDLE

################################################################################################################################################
################################################################################################################################################

def ManualOverride(): # use GUI to manually start/stop game logic using toggle button

    global sight, currentState

    try:
        setTarget, gameStart = gui.getGameState()
        print(f"\nSelected target? ==> {setTarget}\n")
        print(f"\nGame start? ==> {gameStart}\n")
        sight.selectTarget(setTarget)

        if gameStart == False: # keep robot idle
            currentState = State.IDLE
        
        if gameStart == True and currentState == State.IDLE: # unless game start signal is received
            currentState = State.FIND
    
    except Exception as e:
        
        print("\nAn error has occurred:\n")
        
        print(e)

################################################################################################################################################
################################################################################################################################################

def GameLogic(stateSwitch):
    
    startTime = time.time()
    
    # couple things to help assess code execution performance
    countCEFR = 0
    frameCount = 0
    frame = 0
    fps = 0

    global currentState
    
    try:
        while True:
            
            ManualOverride()

            keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sight.FrameProcessor(eyeCam.pipeline, eyeCam.cameraX, eyeCam.cameraY)
            
            print(f"\nRobot state: {currentState}\n")
            
            currentState = switcher.get(currentState)(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
            
            if cv2.waitKey(1) & 0xFF == ord(' '): # press space for emergency stop terminate
                currentState = State.IDLE
                currentState = switcher.get(currentState)(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
                move.allStop()
                break


            frameCount +=1
            frame += 1

            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - startTime)
                startTime = end
                print(f"\nFPS: {fps}, framecount: {frameCount}\n")


            # how many times per second does this game logic iterate thru
            countCEFR += 1 # code iterand: add 1 each iteration cycle
            endTime = time.time()
            
            if(endTime - startTime) > 1: # code execution rate
                print(f"\nCER = {countCEFR/(endTime - startTime)}\n") # calculate and print CER
                countCEFR = 0 # reset counter
                startTime = time.time() # reset timer

    
    except KeyboardInterrupt:        
        eyeCam.stopAllStreams()
    
    except Exception as e:
        print(e)
        raise

    finally:
        eyeCam.stopAllStreams()
        cv2.destroyAllWindows()

GameLogic(stateSwitch)