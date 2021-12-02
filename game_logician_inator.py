from enum import Enum
import time
import cv2
import numpy as np
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
delayTime = 0.005
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

def Find(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):


    #if keypointCount >= 1:
    if ballY is not None and keypointCount >= 1 and throwIterand == 0:
        Drive(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
        return State.DRIVE
    elif throwIterand != 0:
        print("BYPASSED GAME LOGIC STRAIGHT TO THROWING")
        Throw(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
        return State.THROW
    else: # spin until stuff found
        print("I AM SPINNING ROUND ROUND")
        robotSpeed = 0
        robotDirectionAngle = 0
        robotAngularVelocity = 35
        throwerRelativeRPM = 0
        failsafe = 0
        move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)

    # if any balls found, set state DRIVE

    # or set state FIND
    return State.FIND

################################################################################################################################################
################################################################################################################################################

def Drive(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):

    '''
    start by confirming that balls are there
    then check if can go straight to aim state
    if not, check if are aligned with ball already
    if are, go straight to it
    otherwise, align with ball first
    if you lost sight of the ball, 
    go and find another ==> state.find   

    '''
    speed = 30
    throwerRelativeRPM = 0
    failsafe = 0

    if keypointCount > 0:
        if (250 <= ballX <= 390) and (ballY > 300): # distance to ball condition for aim to start
            print("I AM GOING TO AIM NOW")
            Aim(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
            return State.AIM

        elif (250 <= ballX <= 390):
            # when aligned, go to ball
            print(f"ALIGNED WITH BALL, CLOSING IN NOW")
            speed = 65
            robotDirectionAngle = 90
            robotAngularVelocity = 0
            throwerRelativeRPM = 0
            #robotSpeed = (((ballY - (FrameY/2)))/(FrameY))*speed
            robotSpeed = ((450 - ballY)/(FrameY))*speed
            failsafe = 0
            move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        else:
            print(f"I AM ALIGNING WITH THE BALL NOW")
            # orient with ball
            speed = 150
            #forwardSpeed = ((450 - ballY)/FrameY)*speed
            forwardSpeed = 0
            sideSpeed = 0 # ((320 - ballX)/FrameX)*speed # << try this
            rotationSpeed = ((320 - ballX)/FrameX)*speed#* np.sign(((320 - ballX)/FrameX)*speed)
            #rotationSpeed = 0
            throwerRelativeRPM = 0
            failsafe = 0
            move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)

    elif keypointCount <= 0 or None:
        print("I LOST THE BALL, GOING TO FIND IT AGAIN")
        return State.FIND

################################################################################################################################################
################################################################################################################################################

def Aim(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):
    
    '''
    start by confirming you are aligned with ball (again)
    then check if basket is in frame
    if it is not, orbit the ball at a fixed distance until basket is in frame.
    if basket IS in frame, check for alignment to throw first,
    if ready for throwing, ==> state Throw...
    if not aligned, align: align for distance from ball
    align ball with robot mid axis
    or default to aligning with basket
    if in initial check for ball alignment ball is misaligned
    then align again with ball
    '''

    failsafe = 0
    speed = 90
    forwardSpeed = 0
    rotationSpeed = 0
    sideSpeed = 0
    
    if (270 <= ballX <= 370):    
        
        basketNotInFrame = (basketCenterX == None)
        
        if basketNotInFrame: # orbit around ball until basket found            
            speed = 80      
            sideSpeed = ((480 - ballY)/FrameY)*speed
            rotationSpeed = ((480 - ballY)/FrameY)*speed            
            print("\t\tBasket not found, searching for basket.\n")
            throwerRelativeRPM = 0
            failsafe = 0
            move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)

        else: # basket is there, align with basket centre
            print("I AM ABOUT TO THROW THE BALL")       
            if (300 <= basketCenterX <= 340) and (ballY >= 300) and (290 <= ballX <= 350):
                #print("Throw")
                Throw(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
                return State.THROW       
            
            elif (290 <= ballX <= 350):
                speed =  40
                sideSpeed = ((480-ballY)/FrameY)*speed * np.sign(320 - basketCenterX)
                forwardSpeed = 0 #small +
                rotationSpeed = ((480-ballY)/FrameY)*speed * np.sign(320 - basketCenterX)
                throwerRelativeRPM = 0
                failsafe = 0 
                move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)

            else: #Align basket
                speed = 60
                sideSpeed = 0
                forwardSpeed = 0 #small +
                rotationSpeed = ((120 * np.sign(320 - ballX))/FrameX)*speed
                throwerRelativeRPM = 0
                failsafe = 0
                move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
    
    else: #allign ball to the middle
        speed = 60
        forwardSpeed = 0 #small
        sideSpeed = 0
        rotationSpeed = ((200 * np.sign(320 - ballX))/FrameX)*speed
        throwerRelativeRPM = 0
        failsafe = 0
        move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)

    return State.AIM

################################################################################################################################################
################################################################################################################################################

throwIterand = 0

def Throw(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):

    global throwIterand
    if throwIterand >= 40: # revert to state find ball after throwing
        throwIterand = 0
        return State.FIND

    if basketDistance != None:
        failsafe = 0
        #if keypointCount <= 0:
        # get thrower speed for throw
        throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(basketDistance) # change this function
        print(f"THROWING iteration: {throwIterand} with thrower at: {throwerRelativeRPM}")
        
        # using 91 as the robot was not going exactly straight
        move.omniDirect(20, 91, 0, throwerRelativeRPM, failsafe) # move forward and throw ball

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
            #print("State = FIND")

    except Exception as e:

        print("\nAn error has occurred:\n")

        print(e)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
bufferSize = 120 # this controls how many variable the buffer stores, 120 ~= 2[s]
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
bufferDict = {
    "keypointCount": [],
    "ballX": [],
    "ballY": [],
    "basketCenterX": [],
    "basketCenterY": [],
    "basketDistance": []
} # rolling buffer dictionary
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
            print(f"\t\t\t\t\t{currentState}")
            keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sight.ProcessFrame(eyeCam.pipeline, eyeCam.cameraX, eyeCam.cameraY, show=True)
            keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sanityBuffer(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)

            # test this solution: ...if it fails, use the above code... if buffer fcn fails, bypass it until fixed
            #keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sanityBuffer(sight.ProcessFrame(eyeCam.pipeline, eyeCam.cameraX, eyeCam.cameraY, show=True))

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

            #print(f"ballX: {ballX} ballY: {ballY}\n")
            #print(f"basketCenterX: {basketCenterX} basketDistance: {basketDistance}\n")
            currentState = stateSwitch.get(currentState)(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)

            #print(f"{currentState}")
            '''
            print(f"half of frame X: {FrameX/2} == middle of entire frameX: {FrameX}?")
            '''
            #print(f"BallX  = {ballX} ballY = {ballY}")
            '''
            if ballX != None:
                print(f"BallX  == middle of FrameX? {((FrameX/2)-30) <= ballX <= ((FrameX/2)+30)}")
                print(f"BallX - middle of FrameX = {(FrameX/2) - ballX}")
                print(f"BallX - middle of FrameX / FrameX = {((FrameX/2) - ballX)/FrameX}")
                print(f"BallX - middle of FrameX / FrameX = {(((FrameX/2) - ballX)/FrameX)*20}")
            '''
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
                #print(f"\nFPS: {fps}\n")
                frameCount = 0


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
