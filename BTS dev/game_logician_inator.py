import time
import cv2
import numpy as np
import maneuver_inator as move
import thrower_inator as throw
import see_everything_inator as eyes
import threading
import math
from enum import Enum

'''
This is the Main Game Logic module for the B T S test robot. W.I.P.!
'''

################################################################################################################################################
################################################################################################################################################

# enum class with all key State definitions
class State(Enum):
    IDLE = 0 # idling State
    FIND = 1 # finding ball State ( rotate until ball(s) found )
    DRIVE = 2 # drive to found ball
    HOLD = 3 # not implemented but after ball certain distance from bot, collect ball
    AIM = 4 # orbit/rotate until aligned with basket centre
    THROW = 5 # execute throw (if hold used, may need 2 throws)
    CATCH = 6
    AVOID = 7

class GameLogic:
    def __init__(self, sharedData):
        self.sharedData = sharedData
        self.eyeCam = eyes.RealSenseCameraManager() # fully preconfigures camera using the class constructor call
        self.sight = eyes.frameProcessor(sharedData["targetColor"]) # create image core processing instance
        self.currentState = State.IDLE # default starting State
        self.frameX = self.eyeCam.cameraX
        self.frameY = self.eyeCam.cameraY
        self.basketCenterX  = -1
        self.ballX = None
        self.ballY = None
        self.delayTime = 0.005
        self.throwIterand = 0  
        self.lastBasketDist = 0
        self.dataTimeOut = -1
        self.obstacle = False
        self.noBallCounter = 0
        self.aimRotationSign = -1
        self.StateSwitch = {
            State.IDLE: self.Idle,
            State.FIND: self.Find,
            State.DRIVE: self.Drive,
            State.HOLD: self.Hold,
            State.AIM: self.Aim,
            State.THROW: self.Throw,
            State.CATCH: self.Catch,
            State.AVOID: self.Avoid
        }
        self.stateAfterAvoid = State.FIND
        self.lastIsObstacleClose = 1
        
    def Hold(self):
        baseSpeedRotation = 200
        if(self.basketDistance is not None):
            self.lastBasketDist = self.basketDistance
            self.dataTimeOut = 10
        else:
            self.dataTimeOut -=1
        if(self.isObstacleClose != 0):
            self.stateAfterAvoid = State.HOLD
            self.currentState = State.AVOID           
        elif((self.lastBasketDist<2.0 and self.lastBasketDist>1) or self.dataTimeOut < 0):
            move.omniPlanar(0, 0, 0, 0, 0)
            time.sleep(0.3)
            for i in range(40):
                move.omniPlanar(0, 0, 0, 200, 0)             
                time.sleep(0.02) #wait for ball throw
            self.currentState =  State.FIND
            self.dataTimeOut =  -1
        else:
            if(self.lastBasketDist>1.5):
                frontSpeed = min(100,max(60, (self.lastBasketDist - 2.0)/10))
            elif (self.lastBasketDist<1.2):
                frontSpeed = max(-100,min(-60, (self.lastBasketDist - 1.2)/10))
            if self.basketCenterX == -1:            
                rotationSpeed = 0      
            else:                
                rotationSpeed = (self.basketCenterX-20 - self.eyeCam.cameraX/2 )/self.eyeCam.cameraX * baseSpeedRotation
            move.omniPlanar(0, frontSpeed, -rotationSpeed, 0, 0)
        
    def Avoid(self):
        #print("State AVOID")
        if (self.isObstacleClose != 0):
            robotSpeed = 0
            robotDirectionAngle = 0
            robotAngularVelocity = 120#np.sign(self.isObstacleClose) * 60
            self.lastIsObstacleClose = self.isObstacleClose
            throwerRelativeRPM = 0
            failsafe = 0
            move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
            self.currentState = State.AVOID
        else:
            for i in range(20):
                if (self.isObstacleClose != 0):
                    self.currentState = State.AVOID
                    return
                move.omniPlanar(0, 0, np.sign(self.lastIsObstacleClose) * 120, 0, 0) # move forward and throw ball
                time.sleep(0.02) #parallel !!!!
            for i in range(40):
                if (self.isObstacleClose != 0):
                    self.currentState = State.AVOID
                    return
                if(self.keypointCount > 0):
                    self.currentState = State.FIND
                    return
                move.omniPlanar(0, 90, 0, 0, 0) # move forward and throw ball
                time.sleep(0.02) #parallel !!!!
            self.currentState = self.stateAfterAvoid
        #print("AVOID")
    
    def Aim(self):
        #Base code is ok, but it may occure that distX of ball is 290 and dist of basket is 330 and both will pass. With lower limits aiming is prety good but slow
        #Is inspired by KOBI code with some ajustment too formulas
        #Anyway robot movemnents on ultra slow speed must be improved. 
        if self.ballY is None:
            self.currentState = State.FIND
            return
        curBallY = self.ballY
        curBallX = self.ballX 
        curBasketCenterX = self.basketCenterX       
        baseSpeedSide = 50
        baseSpeedForward = 150 
        baseSpeedRotation = 150 
        frontSpeed = (440 - curBallY)/self.eyeCam.cameraY * baseSpeedForward
        if(frontSpeed<20):
            frontSpeed = 20
        if self.basketCenterX is None or self.basketCenterX == -1:            
            sideSpeed = self.aimRotationSign*baseSpeedSide       
        else:
            sideSpeed = (curBasketCenterX - curBallX)/self.eyeCam.cameraX * baseSpeedSide
            if(abs(sideSpeed)<15):
                sideSpeed = np.sign(sideSpeed)*15
        rotationSpeed = (curBallX-10 - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * baseSpeedRotation * 3
        #if(abs(rotationSpeed)>5):
        #    rotationSpeed =  np.sign(sideSpeed)*20
        move.omniPlanar(-sideSpeed, frontSpeed, -rotationSpeed, 0, 0)
        curBasketDist = self.basketDistance
        minThrowError = 12
        #max_throw_distance = 5.25
        basketErrorX = curBallX - curBasketCenterX
        ballErrorX = curBallX - self.eyeCam.cameraX/2
        
        if(curBasketDist != None):
            self.lastBasketDist = curBasketDist
            isBasketErrorXSmallEnough = abs(basketErrorX) < minThrowError
            isBallErrorXSmallEnough = abs(ballErrorX) < minThrowError*3
            #print(basketErrorX, minThrowError)
            if isBasketErrorXSmallEnough and isBallErrorXSmallEnough and curBallY>410 or curBallY>475:
                #if(curBasketDist < 800): #too close                    
                #    print("Borrow ball and move backward")
                #    self.currentState = State.CATCH
                #if(curBasketDist > 3.5):
                #    print("Borrow ball and move forward")
                #    self.currentState = State.CATCH
                #else:
                print("Throw")
                self.currentState = State.THROW    
        
    def Find(self):
        #if keypointCount >= 1:
        if (self.isObstacleClose != 0):
            self.noBallCounter = 0
            self.currentState = State.AVOID
        elif self.ballY is not None and self.keypointCount >= 1 and self.throwIterand == 0:
            #self.noBallCounter = 0
            self.currentState = State.DRIVE
        elif self.throwIterand != 0:
            self.noBallCounter = 0
            print("BYPASSED GAME LOGIC STRAIGHT TO THROWING")
            self.currentState =  State.THROW
        else: # spin until stuff found
            #TO DO dealing with oponent
            #print(self.noBallCounter)
            #print(self.noBallCounter > 3000)
            if( self.noBallCounter > 400):
                self.noBallCounter = 0
                self.stateAfterAvoid = State.FIND
                self.currentState = State.AVOID
                return
            #print("I AM SPINNING ROUND ROUND")
            robotSpeed = 0
            robotDirectionAngle = 0
            robotAngularVelocity = np.sign(self.lastIsObstacleClose)*180
            throwerRelativeRPM = 0
            failsafe = 0
            move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
            self.currentState = State.FIND
            self.noBallCounter +=1
            time.sleep(0.001)
    
    def Drive(self):

        '''
        start by confirming that balls are there
        then check if can go straight to aim State
        if not, check if are aligned with ball already
        if are, go straight to it
        otherwise, align with ball first
        if you lost sight of the ball, 
        go and find another ==> State.find   

        '''
        speed = 50
        throwerRelativeRPM = 0
        failsafe = 0
        if (self.noBallCounter > 0):
            self.noBallCounter -= 30
        if (self.isObstacleClose != 0):
            self.noBallCounter = 0
            self.currentState = State.AVOID
            self.stateAfterAvoid = State.FIND
        elif self.keypointCount > 0:
            if (270+10 <= self.ballX <= 370+10) and (self.ballY > 280): # distance to ball condition for aim to start
                #print("I AM GOING TO AIM NOW")
                self.currentState = State.AIM
                #self.currentState = State.FIND
            else:
                #print(f"I AM ALIGNING WITH THE BALL NOW")
                # orient with ball
                speed = 130
                forwardSpeed =  ((480 - self.ballY)/self.frameY)*speed#move.calculateRelativeSpeed((450 - self.ballY), self.frameY, 5, 200,4,20); #  (deltaFactor, maxDeltaVal, minDeltaVal, maxDeltaSpeed, minAllowedSpeed, maxAllowedSpeed) self.frameY)*speed
                #forwardSpeed = 0
                sideSpeed = 0#move.calculateRelativeSpeed((320 - self.ballX), self.frameX, 5, 200,4,20)#((320 - self.ballX)/self.frameX)*speed#move.calculateRelativeSpeed((320 - self.ballX), self.frameX, 5, 200,4,20); #((320 - self.ballX)/self.frameX)*speed # << try this
                rotationSpeed = ((320 - self.ballX)/self.frameX)*3*speed#move.calculateRelativeSpeed((320 - self.ballX), self.frameX, 5, 200,4,20); #((320 - self.ballX)/self.frameX)*2*speed#* np.sign(((320 - ballX)/FrameX)*speed)
                #rotationSpeed = 0
                throwerRelativeRPM = 0
                failsafe = 0
                move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)

        elif self.keypointCount <= 0 or None:
            print("I LOST THE BALL, GOING TO FIND IT AGAIN")
            self.currentState = State.FIND
            
    def Catch(self):
        #global throwIterand
        #throwIterand = 40
        #if throwIterand >= 40: # revert to State find ball after throwing
        #    throwIterand = 0
        #    return State.FIND
        throwerRelativeRPM = 60
        baseSpeedSide = 30
        baseSpeedRotation = 200
        frontSpeed = 30
        sideSpeed = 0
        rotationSpeed = 0
        failsafe = 0
        print("in Throw")
        curBallX = self.eyeCam.cameraX/2
        curBallY = 440
        for i in range(40):             
            if self.ballX is not None and self.ballY is not None and self.ballY>400: #if <400 not ball in front
                curBallX = self.ballX
                curBallY = self.ballY
            if(curBallY < 450): #if > 440 ball is very close, no need to take it into account
                sideSpeed = (curBallX - self.eyeCam.cameraX/2-10)/self.eyeCam.cameraX * baseSpeedSide
                rotationSpeed = (curBallX - self.eyeCam.cameraX/2-10)/self.eyeCam.cameraX * baseSpeedRotation
            else:
                sideSpeed = 0
                rotationSpeed = 0
            print(f"THROWING iteration: {i} with thrower at: {throwerRelativeRPM}")    
            move.omniPlanar(-sideSpeed, frontSpeed, -rotationSpeed, throwerRelativeRPM, failsafe) # move forward and throw ball
            time.sleep(0.05) #parallel !!!!
        self.dataTimeOut = 10
        self.currentState = State.HOLD


    def Throw(self):
        #global throwIterand
        #throwIterand = 40
        #if throwIterand >= 40: # revert to State find ball after throwing
        #    throwIterand = 0
        #    return State.FIND
        move.omniPlanar(0, 0, 0, 0, 0)
        time.sleep(0.2)
        throwerRelativeRPM = 300
        baseSpeedSide = 15
        baseSpeedRotation = 100
        frontSpeed = 40
        sideSpeed = 0
        rotationSpeed = 0
        failsafe = 0
        print("in Throw")
        curBasketCenterX = self.eyeCam.cameraX/2
        curBallX = self.eyeCam.cameraX/2
        curBallY = 440
        i = 0
        #last_5 = np.array([300,300,300,300,300])
        indx = 0
        while(i<70 and curBallY < 450):        
            #print(self.basketDistance)
            if self.basketDistance != None and self.basketDistance != -1:
                throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(self.basketDistance) #Is potentially ok                
                #indx = (indx + 1)%5
            #throwerRelativeRPM = np.median(last_5)
            if self.basketCenterX != -1:
                curBasketCenterX = self.basketCenterX
            if self.ballX is not None and self.ballY is not None and self.ballY>400: #if <400 not ball in front
                curBallX = self.ballX
                curBallY = self.ballY
            if(curBallY < 440): #if > 440 ball is very close, no need to take it into account
                sideSpeed = (curBasketCenterX - curBallX)/self.eyeCam.cameraX * baseSpeedSide
                rotationSpeed = (curBallX - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * baseSpeedRotation
            else:
                sideSpeed = 0
                rotationSpeed = (curBasketCenterX-20 - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * baseSpeedRotation
            i+=1
            move.omniPlanar(-sideSpeed, frontSpeed, -rotationSpeed, throwerRelativeRPM, failsafe) # move forward and throw ball
            time.sleep(0.01) #parallel !!!!
        frontSpeed = 30           
        for i in range(60):
            if self.basketDistance != None and self.basketDistance != -1:
                throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(self.basketDistance) #Is potentially ok   
            sideSpeed = 0
            rotationSpeed = (curBasketCenterX-20 - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * baseSpeedRotation
            #print(f"THROWING iteration: {i} with thrower at: {throwerRelativeRPM}")    
            move.omniPlanar(-sideSpeed, frontSpeed, -rotationSpeed, throwerRelativeRPM, failsafe) # move forward and throw ball
            time.sleep(0.01) #parallel !!!!
        self.currentState = State.FIND

################################################################################################################################################
################################################################################################################################################

    def Idle(self):

        move.allStop()

        self.currentState = State.IDLE
    
    def infiniteGameLoop(self):
        if(self.sharedData['gameThreadUp'] == False):
            self.sharedData['gameThreadUp'] = True
            self.currentState = State.FIND
            self.stateAfterAvoid = State.FIND
            while self.sharedData['gameThreadUp']:
                self.StateSwitch.get(self.currentState)()
                #print(self.keypointCount)
                #sight.setTarget(eyes.frameProcessor(sharedData["targetColor"]))
                #self.keypointCount, self.ballX, self.ballY, self.basketCenterX, self.basketCenterY, self.basketDistance = sight.ProcessFrame(eyeCam.processor)
            self.sharedData['gameState'] = -1
            self.sharedData['gameThreadUp'] = False
            move.allStop()
            print('Game loop ends')
        
    def infiniteCameraProcessing(self):
        if( self.sharedData['camThreadUp'] == False):
            self.sharedData['camThreadUp'] = True
            while self.sharedData['camThreadUp']: 
                #start = time.time()
                self.sight.selectTarget(self.sharedData['targetColor'])
                if((self.currentState == State.AIM) or (self.currentState == State.THROW) or (self.currentState == State.FIND)):
                    self.keypointCount, curBallX,curBallY, self.basketCenterX, self.basketCenterY, self.basketDistance, frame, self.isObstacleClose, lastBasketX, isLastOponent = self.sight.ProcessFrame(self.eyeCam.processor, True)
                else:
                    self.keypointCount, curBallX,curBallY, self.basketCenterX, self.basketCenterY, self.basketDistance, frame, self.isObstacleClose, lastBasketX, isLastOponent = self.sight.ProcessFrame(self.eyeCam.processor, False)
                if(curBallX is None or curBallY is None): 
                    if(self.currentState != State.HOLD and self.currentState != State.CATCH and self.currentState != State.AVOID):
                        self.currentState = State.FIND
                else:
                    self.ballX = curBallX
                    self.ballY = curBallY
                if(lastBasketX != -1):
                    if(isLastOponent == True):
                        self.aimRotationSign = np.sign(self.frameX/2 - lastBasketX)
                    else:
                        self.aimRotationSign = -np.sign(self.frameX/2 - lastBasketX)
                #print(self.basketDistance)
                #cv2.imshow('Frame', frame)   
                #if cv2.waitKey(1) == ord('q'):
                #    break
                #end = time.time()
                #print(1/(end-start))
            self.sharedData['camThreadUp'] = False
            print('Camera loop ends')
            self.eyeCam.stopAllStreams()
        
    
    
