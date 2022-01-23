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
    def __init__(self, shared_data):
        self.shared_data = shared_data
        self.eyeCam = eyes.RealSenseCameraManager() # fully preconfigures camera using the class constructor call
        self.sight = eyes.frameProcessor(shared_data["targetColor"]) # create image core processing instance
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
        self.StateAfterAvoid = State.FIND
        
    def Hold(self):
        base_speed_rotation = 200
        if(self.basketDistance is not None):
            self.lastBasketDist = self.basketDistance
            self.dataTimeOut = 10
        else:
            self.dataTimeOut -=1
        if(self.is_obstacle_close):
            self.StateAfterAvoid = State.HOLD
            self.currentState = State.AVOID
        elif((self.lastBasketDist<3000 and self.lastBasketDist>1200) or self.dataTimeOut < 0):
            move.omniPlanar(0, 0, 0, 0, 0)
            time.sleep(0.5)
            for i in range(40):
                move.omniPlanar(0, 0, 0, 400, 0)             
                time.sleep(0.05) #wait for ball throw
            self.currentState =  State.FIND
            self.dataTimeOut =  -1
        else:
            if(self.lastBasketDist>2000):
                front_speed = min(60,max(30, (self.lastBasketDist - 2500)/10))
            elif (self.lastBasketDist<1600):
                front_speed = max(-60,min(-30, (self.lastBasketDist - 1800)/10))
            if self.basketCenterX == -1:            
                rot_spd = 0      
            else:                
                rot_spd = (self.basketCenterX - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * base_speed_rotation
            move.omniPlanar(0, front_speed, -rot_spd, 0, 0)
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
        
    def Avoid(self):
        #print("State AVOID")
        if (self.is_obstacle_close):
            robotSpeed = 0
            robotDirectionAngle = 0
            robotAngularVelocity = 60
            throwerRelativeRPM = 0
            failsafe = 0
            move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
            self.currentState = State.AVOID
        else:
            #print(self.is_obstacle_close)
            #print("Free PATH")
            #for i in range(5):
            #    if (self.is_obstacle_close):
            #        self.currentState = State.AVOID
            #        return
            #    move.omniPlanar(0, 0, 60, 0, 0) # move forward and throw ball
            #    time.sleep(0.05) #parallel !!!!
            for i in range(20):
                if (self.is_obstacle_close):
                    self.currentState = State.AVOID
                    return
                move.omniPlanar(0, 40, 0, 0, 0) # move forward and throw ball
                time.sleep(0.05) #parallel !!!!
            self.currentState = self.StateAfterAvoid
        #print("AVOID")
    
    def Aim(self):
        #Base code is ok, but it may occure that distX of ball is 290 and dist of basket is 330 and both will pass. With lower limits aiming is prety good but slow
        #Is inspired by KOBI code with some ajustment too formulas
        #Anyway robot movemnents on ultra slow speed must be improved. 
        '''
        if self.ballY is None:
            self.currentState = State.FIND
            return
        else:
            curBallY = self.ballY 
        if self.ballX is None:
            self.currentState = State.FIND
            return
        else:
            curBallX = self.ballX        
        base_speed_side = 30
        base_speed_forward = 100
        base_speed_rotation = 200 
        front_speed = (430 - curBallY)/self.eyeCam.cameraY * base_speed_forward
        if self.basketCenterX == -1:            
            side_speed = -(self.eyeCam.cameraX)/self.eyeCam.cameraX * base_speed_side       
        else:           
            side_speed = (self.basketCenterX - curBallX)/self.eyeCam.cameraX * base_speed_side
            if(abs(side_speed)<3):
                side_speed = np.sign(side_speed)*3
        basket_error_x = curBallX - self.basketCenterX
        rot_spd = (curBallX - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * base_speed_rotation
        move.omniPlanar(-side_speed, front_speed, -rot_spd, 0, 0)
        curBasketDist = self.basketDistance
        min_throw_error = 8 #add dependance on curBasketDist
        #max_throw_distance = 5.25
       
        
        if(curBasketDist != None):
            self.lastBasketDist = curBasketDist
            is_basket_error_x_small_enough = abs(basket_error_x) < min_throw_error
            print(basket_error_x, min_throw_error)
            if is_basket_error_x_small_enough:
                if(curBasketDist < 500): #too close                    
                    print("Borrow ball and move backward")
                    self.currentState = State.CATCH
                elif(curBasketDist > 3000):
                    print("Borrow ball and move forward")
                    self.currentState = State.CATCH
                else:
                    print("Throw")
                    self.currentState = State.THROW             
        '''
        if self.ballY is None:
            self.currentState = State.FIND
            return
        else:
            curBallY = self.ballY         
        if self.ballX is None:
            self.currentState = State.FIND
            return
        else:
            curBallX = self.ballX 
        if self.basketCenterX is None:
            self.currentState = State.FIND
            return
        else:
            curBasketCenterX = self.basketCenterX       
        base_speed = 30
        base_speed_rotation = 100 
        front_speed = (430 - curBallY)/self.eyeCam.cameraY * base_speed_rotation
        if self.basketCenterX is None or self.basketCenterX == -1:            
            side_speed = -(self.eyeCam.cameraX)/self.eyeCam.cameraX * base_speed       
        else:
            side_speed = (curBasketCenterX - curBallX)/self.eyeCam.cameraX * base_speed#
            if(abs(side_speed)<3):
                side_speed = np.sign(side_speed)*3
        rot_spd = (curBallX - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * base_speed_rotation * 3
        move.omniPlanar(-side_speed, front_speed, -rot_spd, 0, 0)
        curBasketDist = self.basketDistance
        min_throw_error = 8
        #max_throw_distance = 5.25
        basket_error_x = curBallX - curBasketCenterX
        
        if(curBasketDist != None):
            self.lastBasketDist = curBasketDist
            is_basket_error_x_small_enough = abs(basket_error_x) < min_throw_error
            print(basket_error_x, min_throw_error)
            if is_basket_error_x_small_enough:
                if(curBasketDist < 800): #too close                    
                    print("Borrow ball and move backward")
                    self.currentState = State.CATCH
                elif(curBasketDist > 4000):
                    print("Borrow ball and move forward")
                    self.currentState = State.CATCH
                else:
                    print("Throw")
                    self.currentState = State.THROW    
        
    def Find(self):
        #if keypointCount >= 1:
        if (self.is_obstacle_close):
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
            if( self.noBallCounter > 1300):
                self.noBallCounter = 0
                self.StateAfterAvoid = State.FIND
                self.currentState = State.AVOID
                return
            #print("I AM SPINNING ROUND ROUND")
            robotSpeed = 0
            robotDirectionAngle = 0
            robotAngularVelocity = 60
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
        speed = 30
        throwerRelativeRPM = 0
        failsafe = 0
        if (self.noBallCounter > 0):
            self.noBallCounter -= 1
        if (self.is_obstacle_close):
            self.noBallCounter = 0
            self.currentState = State.AVOID
            self.StateAfterAvoid = State.FIND
        elif self.keypointCount > 0:
            if (250 <= self.ballX <= 390) and (self.ballY > 250): # distance to ball condition for aim to start
                print("I AM GOING TO AIM NOW")
                self.currentState = State.AIM
                #self.currentState = State.FIND
                ''''
                elif (250 <= self.ballX <= 390):
                    # when aligned, go to ball
                    print(f"ALIGNED WITH BALL, CLOSING IN NOW")
                    speed = 65
                    robotDirectionAngle = 90
                    robotAngularVelocity = 0
                    throwerRelativeRPM = 0
                    #robotSpeed = (((ballY - (FrameY/2)))/(FrameY))*speed
                    robotSpeed = ((450 - self.ballY)/(self.frameY))*speed
                    failsafe = 0
                    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
                '''
            else:
                print(f"I AM ALIGNING WITH THE BALL NOW")
                # orient with ball
                speed = 100
                forwardSpeed =  ((430 - self.ballY)/self.frameY)*speed#move.calculateRelativeSpeed((450 - self.ballY), self.frameY, 5, 200,4,20); #  (deltaFactor, maxDeltaVal, minDeltaVal, maxDeltaSpeed, minAllowedSpeed, maxAllowedSpeed) self.frameY)*speed
                #forwardSpeed = 0
                sideSpeed = 0#move.calculateRelativeSpeed((320 - self.ballX), self.frameX, 5, 200,4,20)#((320 - self.ballX)/self.frameX)*speed#move.calculateRelativeSpeed((320 - self.ballX), self.frameX, 5, 200,4,20); #((320 - self.ballX)/self.frameX)*speed # << try this
                rotationSpeed = ((320 - self.ballX)/self.frameX)*2*speed#move.calculateRelativeSpeed((320 - self.ballX), self.frameX, 5, 200,4,20); #((320 - self.ballX)/self.frameX)*2*speed#* np.sign(((320 - ballX)/FrameX)*speed)
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
        base_speed_side = 20
        base_speed_rotation = 200
        front_speed = 30
        side_speed = 0
        rot_spd = 0
        failsafe = 0
        print("in Throw")
        curBallX = self.eyeCam.cameraX/2
        curBallY = 430
        for i in range(30):             
            if self.ballX is not None and self.ballY is not None and self.ballY>400: #if <400 not ball in front
                curBallX = self.ballX
                curBallY = self.ballY
            if(curBallY < 440): #if > 440 ball is very close, no need to take it into account
                side_speed = (curBallX - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * base_speed_side
                rot_spd = (curBallX - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * base_speed_rotation
            else:
                sideSpeed = 0
                rot_spd = 0
            print(f"THROWING iteration: {i} with thrower at: {throwerRelativeRPM}")    
            move.omniPlanar(-side_speed, front_speed, -rot_spd, throwerRelativeRPM, failsafe) # move forward and throw ball
            time.sleep(0.05) #parallel !!!!
        self.dataTimeOut = 10
        self.currentState = State.HOLD


    def Throw(self):
        #global throwIterand
        #throwIterand = 40
        #if throwIterand >= 40: # revert to State find ball after throwing
        #    throwIterand = 0
        #    return State.FIND
        throwerRelativeRPM = 300
        base_speed_side = 20
        base_speed_rotation = 200
        front_speed = 20
        side_speed = 0
        rot_spd = 0
        failsafe = 0
        print("in Throw")
        curBasketCenterX = self.eyeCam.cameraX/2
        curBallX = self.eyeCam.cameraX/2
        curBallY = 430
        for i in range(40):
            #print(self.basketDistance)
            if self.basketDistance != None:
                throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(self.basketDistance) #Is potentially ok      
            if self.basketCenterX != -1:
                curBasketCenterX = self.basketCenterX
            if self.ballX is not None and self.ballY is not None and self.ballY>400: #if <400 not ball in front
                curBallX = self.ballX
                curBallY = self.ballY
            if(curBallY < 440): #if > 440 ball is very close, no need to take it into account
                side_speed = (curBasketCenterX - curBallX)/self.eyeCam.cameraX * base_speed_side
                rot_spd = (curBallX - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * base_speed_rotation
            else:
                sideSpeed = 0
                rot_spd = (curBasketCenterX - self.eyeCam.cameraX/2)/self.eyeCam.cameraX * base_speed_rotation
            print(f"THROWING iteration: {i} with thrower at: {throwerRelativeRPM}")    
            move.omniPlanar(-side_speed, front_speed, -rot_spd, throwerRelativeRPM, failsafe) # move forward and throw ball
            time.sleep(0.05) #parallel !!!!
        self.currentState = State.FIND

################################################################################################################################################
################################################################################################################################################

    def Idle(self):

        move.allStop()

        self.currentState = State.IDLE
    
    def infiniteGameLoop(self):
        if(self.shared_data['gameThreadUp'] == False):
            self.shared_data['gameThreadUp'] = True
            self.currentState = State.FIND
            self.StateAfterAvoid = State.FIND
            while self.shared_data['gameThreadUp']:
                self.StateSwitch.get(self.currentState)()
                #print(self.keypointCount)
                #sight.setTarget(eyes.frameProcessor(shared_data["targetColor"]))
                #self.keypointCount, self.ballX, self.ballY, self.basketCenterX, self.basketCenterY, self.basketDistance = sight.ProcessFrame(eyeCam.processor)
            self.shared_data['gameState'] = -1
            self.shared_data['gameThreadUp'] = False
            move.allStop()
            print('Game loop ends')
        
    def infiniteCameraProcessing(self):
        if( self.shared_data['camThreadUp'] == False):
            self.shared_data['camThreadUp'] = True
            while self.shared_data['camThreadUp']:                
                #start = time.time()            
                self.sight.selectTarget(self.shared_data['targetColor'])
                self.keypointCount, curBallX,curBallY, self.basketCenterX, self.basketCenterY, self.basketDistance, frame, self.is_obstacle_close = self.sight.ProcessFrame(self.eyeCam.processor)
                #if(self.is_obstacle_close):
                    #print("AVOID")
                if(curBallX is None or curBallY is None): 
                    if(self.currentState != State.HOLD and self.currentState != State.CATCH and self.currentState != State.AVOID):
                        self.currentState = State.FIND
                else:
                    self.ballX = curBallX
                    self.ballY = curBallY
                #print(self.keypointCount)
                cv2.imshow('Frame', frame)   
                if cv2.waitKey(1) == ord('q'):
                    break
                #end = time.time()
                #print(1/(end-start))
            self.shared_data['camThreadUp'] = False
            print('Camera loop ends')
            self.eyeCam.stopAllStreams()
        
    
    
