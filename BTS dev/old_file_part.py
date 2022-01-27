#B_T_S Final Code#
#January 26th 2022#

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
#STOLEN FROM KOBI

#END OF STOLEN
################################################################################################################################################
################################################################################################################################################



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
        if (250 <= ballX <= 390) and (ballY > 250): # distance to ball condition for aim to start
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
    #Base code is ok, but it may occure that distX of ball is 290 and dist of basket is 330 and both will pass. With lower limits aiming is prety good but slow
    #In Kobis approach movement is faster but aiming may miss in situations when both ball and basket are on the side (far from cameraX)
    #Anyway robot movemnents on ultra slow speed must be improved.
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
    '''
    failsafe = 0
    speed = 90
    forwardSpeed = 0
    rotationSpeed = 0
    sideSpeed = 0

    if (270 <= ballX <= 370):

        basketNotInFrame = (basketCenterX == -1)

        if basketNotInFrame or 290 > basketCenterX: # orbit around ball until basket found
            speed = 40
            sideSpeed = ((40+480 - ballY)/FrameY)*speed
            rotationSpeed = ((40+480 - ballY)/FrameY)*speed
            print("\t\tBasket not found, searching for basket.\n")
            throwerRelativeRPM = 0
            failsafe = 0
            move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
        elif 350 < basketCenterX: # orbit around ball until basket found
            speed = 40
            sideSpeed = -((40+480 - ballY)/FrameY)*speed
            rotationSpeed = ((40+480 - ballY)/FrameY)*speed
            print("\t\tBasket not found, searching for basket.\n")
            throwerRelativeRPM = 0
            failsafe = 0
            move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
        else: # basket is there, align with basket centre
            print("TARGETING")
            print(basketCenterX)
            print(ballX)
            if (290 <= basketCenterX <= 350) and (ballY >= 400) and (290 <= ballX <= 350):
                print("Throw")
                print(basketDistance)
                Throw(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
                return State.THROW
            elif (290 > ballX < 350):
                print("Ball not in center")
                speed =  10
                sideSpeed = 0 #((480-ballY)/FrameY)*speed * np.sign(320 - basketCenterX)
                forwardSpeed = -5 #small +
                rotationSpeed = speed * np.sign(320 - basketCenterX) #((480-ballY)/FrameY)*
                throwerRelativeRPM = 0
                failsafe = 0
                move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
            else:
                print("Ball too far")
                speed =  30
                sideSpeed = 0
                forwardSpeed = 10 #small +
                rotationSpeed = 0
                throwerRelativeRPM = 0
                failsafe = 0
                move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)


    else: #allign ball to the middle
        speed = 60
        forwardSpeed = -5 #small
        sideSpeed = 0
        rotationSpeed = ((200 * np.sign(320 - ballX))/FrameX)*speed
        throwerRelativeRPM = 0
        failsafe = 0
        move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)
    '''
    #if (300 <= basketCenterX <= 340) and (ballY >= 400) and (300 <= ballX <= 340):
    #    print("Throw")
    #    print(basketDistance)
    #    Throw(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)
    #    return State.THROW
    #STOLEN FROM KOBI
    if basketCenterX == -1:
        delta_x = eyeCam.cameraX
    else:
        delta_x = ballX - basketCenterX
    rot_delta_x = ballX - eyeCam.cameraX/2
    delta_y = 400 - ballY #400 is ok
    front_speed = calc_speed(delta_y, eyeCam.cameraY, 5, 3, 100, 30)
    side_speed = calc_speed(delta_x, eyeCam.cameraX, 5, 4, 100, 20)
    rot_spd = calc_speed(rot_delta_x, eyeCam.cameraX, 5, 3, 100, 40)
    move.omniPlanar(side_speed, front_speed, rot_spd, 0, 0)
    min_throw_error = 8 #1.5
    max_throw_distance = 5.25
    basket_error_x = ballX - basketCenterX#state_data.basket_x - camera.camera_x
    if(basketDistance != None):
        throw_error = max_throw_distance / basketDistance * min_throw_error/1000
        is_basket_too_far = basketDistance > 3000
        is_basket_too_close = basketDistance < 500
        if throw_error < min_throw_error:
                throw_error = min_throw_error
        is_basket_error_x_small_enough = abs(basket_error_x) < throw_error
        print(basket_error_x, throw_error)
        if is_basket_error_x_small_enough:
            print("Throw")
            Throw(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)

    #END OF STOLEN
    return State.AIM

################################################################################################################################################
################################################################################################################################################

throwIterand = 0

def Throw(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance):

    #global throwIterand
    #throwIterand = 40
    #if throwIterand >= 40: # revert to state find ball after throwing
    #    throwIterand = 0
    #    return State.FIND
    throwerRelativeRPM = 300
    print("in Throw")
    for i in range(40):
        print(basketDistance)
        if basketDistance != None:
            failsafe = 0
            #if keypointCount <= 0:
            # get thrower speed for throw
            throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(basketDistance) # change this function
        print(f"THROWING iteration: {i} with thrower at: {throwerRelativeRPM}")
        # using 91 as the robot was not going exactly straight
        move.omniDirect(20, 91, 0, throwerRelativeRPM, failsafe) # move forward and throw ball
        time.sleep(0.05)

    return State.FIND

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

    #print(f"\nGame Start True/False from main file? {gameStart}\n")
    try:
        #print(f"\nSelected target? ==> {setTarget}\n")
        #print(f"\nGame start? ==> {gameStart}\n")
        sight.selectTarget(shared_data["targetColor"])
        currentState = State.IDLE
        if shared_data["gameLogicStart"] == False: # keep robot idle
            currentState = State.IDLE

        elif shared_data["gameLogicStart"] == True and currentState == State.IDLE: # unless game start signal is received
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



def gameLogic(sharedMemory,stateSwitch):
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
            #keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sight.ProcessFrame(eyeCam.processor)
            #keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance = sanityBuffer(keypointCount, ballX, ballY, basketCenterX, basketCenterY, basketDistance)

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
            if (sharedMemory['gameState'] == -1 or (cv2.waitKey(1) & 0xFF == ord('q'))): # q ==> stop all
                sharedMemory['gameState'] = -1
                move.allStop()
                print("Exit infinite loop")
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

def runGameLogic(sharedMemory, gameLogicStart): # start tkinter gui fcn):
    #if(sharedMemory["gameThreadUp"] == False):
    #    sharedMemory["gameThreadUp"] = True
    if gameLogicStart == True:
        print("Game Logic Running!")
        threading.Thread(gameLogic(sharedMemory,stateSwitch)).start()
    setTarget = shared_data["targetColor"]
    gameStart = shared_data["gameLogicStart"]
    print(setTarget, gameStart)
