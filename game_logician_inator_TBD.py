
def Drive(keypointCount, ballX, ballY, frameCenterX, frameCenterY, basketDistance):
    if keypointCount > 0:
        
        delta_x = x - Camera.camera_x/2
        delta_y = 390-y
        minSpeed = 2
        maxSpeed = 50
        minDelta = 5
        
        forwardSpeed = move.calculateRelativeSpeed(delta_y, Camera.camera_y, minDelta, 5, 350, 40)#3 + (480-y)/ 540.0 * 30
        
        sideSpeed = 0
        
        rotationSpeed = move.calculateRelativeSpeed(delta_x, Camera.camera_x, minDelta, minSpeed, 150, 20)#int((x - 480)/480.0 * 25)
                
        move.omniPlanar(sideSpeed, forwardSpeed, rotationSpeed, 0)
        
        if 500 > y > 315 : # How close ball y should be to switch to next state
    
            return State.AIM
    
    if keypointCount <= 0 or None:
    
        return State.FIND

    return State.DRIVE

def Find(keypointCount, ballX, ballY, frameCenterX, frameCenterY, basketDistance):
    
    # rotate left around robot central axis

    robotSpeed = 0
    robotDirectionAngle = 0
    robotAngularVelocity = 60
    throwerRelativeRPM = 0
    failsafe = 0 

    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
    
    # if any balls found, set state DRIVE

    if keypointCount >= 1:
        Drive(keypointCount, ballX, ballY, frameCenterX, frameCenterY, basketDistance)
        return State.DRIVE
    
    # or set state FIND
    return State.FIND


def Aim(keypointCount, ballX, ballY, frameCenterX, frameCenterY, basketDistance):
    
    basketInFrame = center_x is not None
    if x is None :
        return State.FIND

    if not basketInFrame:
        delta_x = Camera.camera_x
    else:
        delta_x = x - center_x

    rot_delta_x = x - Camera.camera_x/2
    
    delta_y = 450 - y
    
    forwardSpeed = move.calculateRelativeSpeed(delta_y, Camera.camera_y, 7, 3, 150, 30)
    
    sideSpeed = move.calculateRelativeSpeed(delta_x, Camera.camera_x, 7, 3, 150, 30)
    
    rotationSpeed = move.calculateRelativeSpeed(rot_delta_x, Camera.camera_x, 7, 2, 150, 30)
    
    print("y", y, "x", x, "center", center_x, "side", sideSpeed, "front", forwardSpeed,"rot", rotationSpeed)   
    
    move.omniPlanar(-sideSpeed, forwardSpeed, -rotationSpeed, 0)
    
    if basketInFrame and 315 <= center_x <= 325 and y >= 410: # Start throwing if ball y is close to robot and basket is centered to camera x
       move.allStop()
       return State.THROW

    return State.AIM


i = 0
def Throw(keypointCount, ballX, ballY, frameCenterX, frameCenterY, basketDistance):
    global i
    if i >= 20:
        i = 0
        return State.FIND
    if keypointCount >= 1:
        basketInFrame = center_x is not None

        if not basketInFrame:
            delta_x = Camera.camera_x
        else:
            delta_x = x - center_x
        rot_delta_x = x - Camera.camera_x/2
        delta_y = 500 - y
        
        minSpeed = 10
        maxSpeed = 30
        minDelta = 6

        throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(basketDistance)
        
        forwardSpeed = move.calculateRelativeSpeed(delta_y, Camera.camera_y, minDelta, minSpeed, 150, maxSpeed)
        
        sideSpeed = move.calculateRelativeSpeed(delta_x, Camera.camera_x, minDelta, minSpeed, 150, maxSpeed)
        
        rotationSpeed = move.calculateRelativeSpeed(rot_delta_x, Camera.camera_x, minDelta, 3, 100, maxSpeed)
        
        move.omniPlanar(-0, forwardSpeed, -0, throwerRelativeRPM)
    
    if keypointCount <= 0:

        throwerRelativeRPM = throw.throwerSpeedFromDistanceToBasket(basketDistance)
        
        move.omniPlanar(0, 10, 0, throwerRelativeRPM)
        
        i += 1
    
    return State.THROW


def Idle(keypointCount, ballX, ballY, frameCenterX, frameCenterY, basketDistance):
    move.allStop()
    return State.IDLE

def ManualOverride(): # use GUI to manually start/stop game logic using toggle button
    #import rc_inator as gui

    global sight, currentState

    try:
        setTarget, gameStart = gui.getGameState()
        
        print(f"\nSelected target? ==> {setTarget}\n")
        
        print(f"\nGame start? ==> {gameStart}\n")
        
        sight.selectTarget(setTarget)

        if gameStart == False: # keep robot state at idle
        
            currentState = State.IDLE
        
        if gameStart == True and currentState == State.IDLE: # unless received game start signal
        
            currentState = State.FIND
    
    except Exception as e:
        print("\nAn error has occurred:\n")
        print(e)