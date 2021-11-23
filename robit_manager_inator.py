from enum import Enum
import time
import cv2
import numpy
import maneuver_inator as move
import thrower_inator as throw
import see_everything_inator as eyes


'''
This is the Main Game Logic module for the B T S test robot. W.I.P.!
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

eyeCam = eyes.RealSenseCameraManager() # fully preconfigures camera

currentState = State.IDLE # default starting state

setTarget = "magenta" #set target basket "blue" or "magenta" (default)

sight = eyes.frameProcessor(setTarget) # create image core processing instance

FrameX = eyeCam.cameraX
FrameY = eyeCam.cameraY
print(FrameX, FrameY)
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

        '''
        side movement: ((ballX - basketCenterX)/FrameX)*speed
        forward movement: ((ballY - midCamFrameY)/FrameY)*speed
        rotating movement: ((ballX - midCamFrameX)/FrameX)*speed
        '''

        speed = 30
        throwerRelativeRPM = 0
        failsafe = 0

        forwardSpeed = ((ballY - FrameY/2)/FrameY)*speed
        rotationSpeed = ((ballX - FrameX/2)/FrameX)*speed
        sideSpeed = 0 # robotSpeedX, robotSpeedY, robotAngularVelocity

        move.omniPlanar(sideSpeed, forwardSpeed, -rotationSpeed, throwerRelativeRPM, failsafe)
        
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
    robotAngularVelocity = 30
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
    '''
    side movement: ((ballX - basketCenterX)/FrameX)*speed
    forward movement: ((ballY - midCamFrameY)/FrameY)*speed
    rotating movement: ((ballX - midCamFrameX)/FrameX)*speed
    '''
    basketInFrame = basketCenterX is not None

    if ballX is None :
        return State.FIND

    speed = 30
    if not basketInFrame: # orbit around ball until basket found
        forwardSpeed = ((ballY - FrameY/2)/FrameY)*speed
        #forwardSpeed = 0
        sideSpeed = ((ballX - basketCenterX/2)/FrameX)*speed
        rotationSpeed = ((ballX - FrameX/2)/FrameX)*speed
        print("Basket not found, searching.\n")
    
    else: # basket is there, align with basket centre

        print("Basket found, aiming.\n")
        forwardSpeed = ((ballY - FrameY/2)/FrameY)*speed
        sideSpeed = ((ballX - basketCenterX/2)/FrameX)*speed
        rotationSpeed = ((ballX - FrameX/2)/FrameX)*speed
    
    throwerRelativeRPM = 0
    failsafe = 0
    
     
    move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
    
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
    
    '''
    basketInFrame = basketCenterX is not None

    if ballX is None :
        return State.FIND

    speed = 10
    if not basketInFrame: # orbit around ball until basket found
        forwardSpeed = ((ballY - FrameY/2)/FrameY)*speed
        #forwardSpeed = 0
        sideSpeed = ((ballX - basketCenterX/2)/FrameX)*speed
        rotationSpeed = ((ballX - FrameX/2)/FrameX)*speed
        print("Basket not found, searching.\n")
    
    else: # basket is there, align with basket centre

        print("Basket found, aiming.\n")
        forwardSpeed = ((ballY - FrameY/2)/FrameY)*speed
        sideSpeed = ((ballX - basketCenterX/2)/FrameX)*speed
        rotationSpeed = ((ballX - FrameX/2)/FrameX)*speed
    '''

    if throwIterand >= 15: # revert to state find ball after throwing
        throwIterand = 0
        return State.FIND
    
    if keypointCount >= 1: # if balls are found, align with basket again
        
        basketInFrame = basketCenterX is not None

        if not basketInFrame:
            forwardSpeed = ((ballY - FrameY/2)/FrameY)*speed # could be 0 but this keeps ball at fixed distance ideally
            sideSpeed = ((ballX - basketCenterX/2)/FrameX)*speed
            rotationSpeed = ((ballX - FrameX/2)/FrameX)*speed
            print("Basket not found, searching.\n")
        
        else:
            print("Basket found, aiming.\n")
            forwardSpeed = ((ballY - FrameY/2)/FrameY)*speed
            sideSpeed = ((ballX - basketCenterX/2)/FrameX)*speed
            rotationSpeed = ((ballX - FrameX/2)/FrameX)*speed

        move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
    
    if keypointCount <= 0:

        throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(basketDistance)
        
        move.omniPlanar(0, 15, 0, throwerRelativeRPM) # move forward and throw ball
        
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
    from rc_inator import targetColor, gameLogicStart
    setTarget = targetColor
    gameStart = gameLogicStart
    #setTarget = "blue"
    #gameStart = True
    #print(f"\nGame Start True/False from main file? {gameStart}\n")
    try:
        #print(f"\nSelected target? ==> {setTarget}\n")
        #print(f"\nGame start? ==> {gameStart}\n")
        sight.selectTarget(setTarget)
        currentState = State.IDLE
        if gameStart == False: # keep robot idle
            currentState = State.IDLE
        
        elif gameStart == True and currentState == State.IDLE: # unless game start signal is received
            currentState = State.FIND
            print("State = FIND")
    
    except Exception as e:
        
        print("\nAn error has occurred:\n")
        
        print(e)

################################################################################################################################################
################################################################################################################################################

stateSwitch = {
    State.IDLE: Idle,
    State.FIND: Find,
    State.DRIVE: Drive,
    State.HOLD: Hold,
    State.AIM: Aim,
    State.THROW: Throw
}

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
            #print("GameLogicRunning")
            ManualOverride()

            keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sight.ProcessFrame(eyeCam.pipeline, eyeCam.cameraX, eyeCam.cameraY)
            
            #print(f"\nHow many balls? {keypointCount}\n")
            #print(f"\nRobot state: {currentState}\n")
            print(f"ballX: {ballX} ballY: {ballY}\n")
            currentState = stateSwitch.get(currentState)(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
            print(f"{currentState}")
            if cv2.waitKey(1) & 0xFF == ord(' '): # press space for emergency stop terminate
                
                move.allStop()
                break


            frameCount +=1
            frame += 1

            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - startTime)
                startTime = end
                #print(f"\nFPS: {fps}, framecount: {frameCount}\n")


            # how many times per second does this game logic iterate thru
            countCEFR += 1 # code iterand: add 1 each iteration cycle
            endTime = time.time()
            
            if(endTime - startTime) > 1: # code execution rate
                #print(f"\nCER = {countCEFR/(endTime - startTime)}\n") # calculate and print CER
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