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

# enum class with all key state definitions
class State(Enum):
    IDLE = 0 # idling state
    FIND = 1 # finding ball state ( rotate until ball(s) found )
    DRIVE = 2 # drive to found ball
    HOLD = 3 # not implemented but after ball certain distance from bot, collect ball
    AIM = 4 # orbit/rotate until aligned with basket centre
    THROW = 5 # execute throw (if hold used, may need 2 throws)

class GameLogic:
    def __init__(self, shared_data):
        self.shared_data = shared_data
        self.eyeCam = eyes.RealSenseCameraManager() # fully preconfigures camera using the class constructor call
        self.sight = eyes.frameProcessor(shared_data["targetColor"]) # create image core processing instance
        self.currentState = State.IDLE # default starting state
        self.FrameX = self.eyeCam.cameraX
        self.FrameY = self.eyeCam.cameraY
        self.delayTime = 0.005
        self.throwIterand = 0        
        self.stateSwitch = {
            State.IDLE: self.Idle,
            State.FIND: self.Find,
            State.DRIVE: self.Drive,
            State.HOLD: self.Hold,
            State.AIM: self.Aim,
            State.THROW: self.Throw
        }
    
    def calc_speed(self,delta, max_delta, min_delta, min_speed, max_delta_speed, max_speed):
        if abs(delta) < min_delta:
            return 0
        delta_div = delta / max_delta
        sign = math.copysign(1, delta_div)
        normalized_delta = math.pow(abs(delta_div), 2) * sign
        speed = normalized_delta * max_delta_speed
        return int(int(speed) if abs(speed) >= min_speed and abs(speed) <= max_speed else max_speed * sign if speed > max_speed else min_speed * sign)
        
    def Hold():
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
        self.currentState =  State.AIM
    
    def Aim(self):
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
        if self.basketCenterX == -1:
            delta_x = self.eyeCam.cameraX
        else:
            delta_x = self.ballX - self.basketCenterX
        rot_delta_x = self.ballX - self.eyeCam.cameraX/2    
        delta_y = 400 - self.ballY #400 is ok    
        front_speed = self.calc_speed(delta_y, self.eyeCam.cameraY, 5, 3, 100, 30)
        side_speed = self.calc_speed(delta_x, self.eyeCam.cameraX, 5, 4, 100, 20)
        rot_spd = self.calc_speed(rot_delta_x, self.eyeCam.cameraX, 5, 3, 100, 40)
        move.omniPlanar(side_speed, front_speed, rot_spd, 0, 0)
        min_throw_error = 8 #1.5
        max_throw_distance = 5.25
        basket_error_x = self.ballX - self.basketCenterX#state_data.basket_x - camera.camera_x
        if(self.basketDistance != None):
            throw_error = max_throw_distance / self.basketDistance * min_throw_error/1000
            is_basket_too_far = self.basketDistance > 3000
            is_basket_too_close = self.basketDistance < 500
            if throw_error < min_throw_error:
                    throw_error = min_throw_error
            is_basket_error_x_small_enough = abs(basket_error_x) < throw_error
            print(basket_error_x, throw_error)
            if is_basket_error_x_small_enough:
                print("Throw")
                self.currentState = State.Throw
                return
          
        #END OF STOLEN
        self.currentState = State.AIM
        
        
    def Find(self):
        #if keypointCount >= 1:
        print('Here')
        if self.ballY is not None and self.keypointCount >= 1 and self.throwIterand == 0:
            self.currentState = State.DRIVE
        elif self.throwIterand != 0:
            print("BYPASSED GAME LOGIC STRAIGHT TO THROWING")
            self.currentState =  State.THROW
        else: # spin until stuff found
            print("I AM SPINNING ROUND ROUND")
            robotSpeed = 0
            robotDirectionAngle = 0
            robotAngularVelocity = 35
            throwerRelativeRPM = 0
            failsafe = 0
            move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
            self.currentState = State.FIND
    
    def Drive(self):

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

        if self.keypointCount > 0:
            if (250 <= self.ballX <= 390) and (self.ballY > 250): # distance to ball condition for aim to start
                print("I AM GOING TO AIM NOW")
                self.currentState = State.AIM

            elif (250 <= self.ballX <= 390):
                # when aligned, go to ball
                print(f"ALIGNED WITH BALL, CLOSING IN NOW")
                speed = 65
                robotDirectionAngle = 90
                robotAngularVelocity = 0
                throwerRelativeRPM = 0
                #robotSpeed = (((ballY - (FrameY/2)))/(FrameY))*speed
                robotSpeed = ((450 - self.ballY)/(self.FrameY))*speed
                failsafe = 0
                move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
            else:
                print(f"I AM ALIGNING WITH THE BALL NOW")
                # orient with ball
                speed = 150
                #forwardSpeed = ((450 - ballY)/FrameY)*speed
                forwardSpeed = 0
                sideSpeed = 0 # ((320 - ballX)/FrameX)*speed # << try this
                rotationSpeed = ((320 - self.ballX)/self.FrameX)*speed#* np.sign(((320 - ballX)/FrameX)*speed)
                #rotationSpeed = 0
                throwerRelativeRPM = 0
                failsafe = 0
                move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, throwerRelativeRPM, failsafe)

        elif self.keypointCount <= 0 or None:
            print("I LOST THE BALL, GOING TO FIND IT AGAIN")
            self.currentState = State.FIND
    
    def Throw(self):

        #global throwIterand
        #throwIterand = 40
        #if throwIterand >= 40: # revert to state find ball after throwing
        #    throwIterand = 0
        #    return State.FIND
        throwerRelativeRPM = 300
        print("in Throw")
        for i in range(40):
            print(self.basketDistance)
            if self.basketDistance != None:
                failsafe = 0
                #if keypointCount <= 0:
                # get thrower speed for throw
                throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(self.basketDistance) # change this function
            print(f"THROWING iteration: {i} with thrower at: {throwerRelativeRPM}")        
            # using 91 as the robot was not going exactly straight
            move.omniDirect(20, 91, 0, throwerRelativeRPM, failsafe) # move forward and throw ball
            time.sleep(0.05)

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
            while self.shared_data['gameThreadUp']:
                self.stateSwitch.get(self.currentState)()
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
                #print(self.shared_data['targetColor'])
                self.sight.selectTarget(self.shared_data['targetColor'])
                self.keypointCount, self.ballX, self.ballY, self.basketCenterX, self.basketCenterY, self.basketDistance, frame = self.sight.ProcessFrame(self.eyeCam.processor)
                #print(self.keypointCount)
                cv2.imshow('Frame', frame)   
                if cv2.waitKey(1) == ord('q'):
                    break
                #end = time.time()
                #print(1/(end-start))
            self.shared_data['camThreadUp'] = False
            print('Camera loop ends')
            self.eyeCam.stopAllStreams()
        
    
    
