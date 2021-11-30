from enum import Enum
import time
import cv2
import numpy
import maneuver_inator as move
import thrower_inator as throw
import see_everything_inator as eyes
import threading
import rc_inator as gui

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

eyeCam = eyes.RealSenseCameraManager() # fully preconfigures camera using the class constructor call
setTarget = "magenta" #set target basket "blue" or "magenta" (default value)
sight = eyes.frameProcessor(setTarget) # create image core processing instance
currentState = State.IDLE # default starting state
FrameX = eyeCam.cameraX
FrameY = eyeCam.cameraY

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

'''
have memory of recent ball positions & confirm they existed
could use up to 10 frames        
relative motion straight towards the ball
side movement: ((ballX - basketCenterX)/FrameX)*speed
forward movement: ((ballY - midCamFrameY)/FrameY)*speed
rotating movement: ((ballX - midCamFrameX)/FrameX)*speed
'''
def Drive(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    
    if keypointCount > 0:
        speed = 30
        throwerRelativeRPM = 0
        failsafe = 0
        # go to ball
        forwardSpeed = (((ballY - FrameY))/FrameY)*speed
        rotationSpeed = (((ballX - FrameX))/FrameX)*speed
        #forwardSpeed = 0
        sideSpeed = 0 # robotSpeedX, robotSpeedY, robotAngularVelocity

        move.omniPlanar(sideSpeed, forwardSpeed, -rotationSpeed, throwerRelativeRPM, failsafe)
        
        if 480 > ballY > 350 : # distance to ball condition for aim to start
            #print("\t\t\t\t\t\t\tTHIS IS EXECUTING")
            Aim(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
            #move.allStop()
            return State.AIM
    
    elif keypointCount <= 0 or None:
    
        return State.FIND

    return State.DRIVE

################################################################################################################################################
################################################################################################################################################

def Find(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    
    # rotate left around robot central axis
    '''
    if you dont find ball in a few rotations, 
    try moving towards a basket
    then repeat rotations
    '''
    robotSpeed = 0
    robotDirectionAngle = 0
    robotAngularVelocity = 50
    throwerRelativeRPM = 0
    failsafe = 0 
    #print("\t\tSTATE FIND")
    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
    
    # if any balls found, set state DRIVE

    #if keypointCount >= 1:
    if ballX is not None and keypointCount >= 1:
        #print("\t\t\tGO TO STATE DRIVE")
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
    basketInFrame = basketCenterX is not 'NoneType'
    #print("\t\tSTATE AIM ", basketInFrame)

    #if ballX is None :
    #    return State.FIND

    speed = 30

    if basketInFrame: # orbit around ball until basket found
        forwardSpeed = ((ballY - FrameY)/FrameY)*speed
        #sideSpeed = ((ballX - basketCenterX)/FrameX)*speed
        sideSpeed = 0
        rotationSpeed = ((ballX - FrameX)/FrameX)*speed
        print("\t\tBasket not found, searching for basket.\n")
        throwerRelativeRPM = 0
        failsafe = 0
        move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
    
    else: # basket is there, align with basket centre
        print("\t\tBasket found, aiming.\n")
        forwardSpeed = ((ballY - FrameY)/FrameY)*speed
        sideSpeed = ((ballX - basketCenterX)/FrameX)*speed
        rotationSpeed = ((ballX - FrameX)/FrameX)*speed
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
    '''
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
    '''
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
'''
# utility functions: 
# >>manual override ==> get gamestart from gui and/or from referree server
# >>sanityBuffer ==> rolling buffer that return image processing output on each call

'''
def ManualOverride(): # use GUI to manually start/stop game logic using toggle button

    global sight, currentState
    #from rc_inator import targetColor, gameLogicStart
    setTarget = gui.targetColor
    gameStart = gui.gameLogicStart

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

bufferSize = 10 # this controls how many variable the buffer stores
bufferDict = {
    "keypointCount": [],
    "ballX": [],
    "ballY": [],
    "basketCenterX": [],
    "basketCenterY": [],
    "basketDistance": []
} # buffer dictionary
# if you add output values from image processing, don't forget to add them to either dict as well!
def sanityBuffer(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    outputDict = {
    "keypointCount": keypointCount,
    "ballX": ballX,
    "ballY": ballY,
    "basketCenterX": basketCenterX,
    "basketCenterY": basketCenterY,
    "basketDistance": basketDistance
    } # output dictionary
    for key in bufferDict.keys():
        bufferDict[key].append(outputDict[key])            
        if len(bufferDict[key]) > bufferSize:
            del bufferDict[key][0]
        summaErrata = []
        for val in bufferDict[key]:
            if val != 'TypeNone':
                summaErrata.append(val)
        if len(summaErrata) > 0:
            outputDict[key] = summaErrata[-1]
        else:
            outputDict[key] = 'TypeNone'
    return outputDict['keypointCount'], outputDict['ballX'], outputDict['ballY'], outputDict['basketCenterX'], outputDict['basketCenterY'], outputDict['basketDistance']
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

            #keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sight.ProcessFrame(eyeCam.pipeline, eyeCam.cameraX, eyeCam.cameraY, show=True)
            #keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sanityBuffer(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
            
            # test this solution: ...if it fails, use the above code... if buffer fcn fails, bypass it until fixed
            keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sanityBuffer(sight.ProcessFrame(eyeCam.pipeline, eyeCam.cameraX, eyeCam.cameraY, show=True))
            
            '''
            # may perhaps consider adding an encapsulation layer for the returned values to neatify the code a bit...
            gameVals = [keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance]
            keypointCount = gameVals[0]
            ballX = gameVals[1]
            ballY = gameVals[2]
            basketCenterX = gameVals[3]
            basketCenterY = gameVals[4]
            basketDistance = gameVals[5]
            fieldLine = gameVals[6]
            '''
            #print(f"\nHow many balls? {keypointCount}\n")
            #print(f"\nRobot state: {currentState}\n")
            
            print(f"ballX: {ballX} ballY: {ballY}\n")
            
            currentState = stateSwitch.get(currentState)(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
            
            print(f"{currentState}")
            
            if cv2.waitKey(1) & 0xFF == ord('q'): # q ==> stop all    
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
        #eyeCam.stopAllStreams()
        cv2.destroyAllWindows()

def runGameLogic(gameLogicStart): # start tkinter gui fcn):
    if gameLogicStart == True:
        print("Game Logic Running!")
        threading.Thread(GameLogic(stateSwitch)).start()
    setTarget = gui.targetColor
    gameStart = gui.gameLogicStart
    print(setTarget, gameStart)
