import maneuver_inator as move
import thrower_inator as throw
import cv2
import numpy as np
from tkinter import *
import time

'''
This is the Graphical User Interface module for the B T S test robot.

When run, a window is created featuring instructions, buttons and a text log of command inputs.

'''


################################################################################################################################################
################################################################################################################################################

def keyInput(event):
    delayTime = 0.01
    throwerRelativeRPM = 0
    robotSpeed = 30

    keyPress = event.keysym.lower()

    if keyPress == 'w':
        msg = "Moving forward.\n"
        cmdLog.insert(0.0, msg)
        robotDirectionAngle = 90
        robotAngularVelocity = 0
            
        move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'd':
        msg = "Moving right.\n"
        cmdLog.insert(0.0, msg)
        robotDirectionAngle = 0
        robotAngularVelocity = 0
            
        move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 's':
        msg = "Moving back.\n"
        cmdLog.insert(0.0, msg)
        robotDirectionAngle = 270
        robotAngularVelocity = 0
            
        move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'a':
        msg = "Moving left.\n"
        cmdLog.insert(0.0, msg)
        robotDirectionAngle = 180
        robotAngularVelocity = 0

        move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'e':
        msg = "Spinning right.\n"
        cmdLog.insert(0.0, msg)
        robotSpeed = 0
        robotDirectionAngle = 0
        robotAngularVelocity = -30
            
        move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'q':
        msg = "Spinning left.\n"
        cmdLog.insert(0.0, msg)
        robotSpeed = 0
        robotDirectionAngle = 0
        robotAngularVelocity = 30
            
        move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'space':
        msg = "Stopped.\n"
        cmdLog.insert(0.0, msg)

        move.allStop()
        time.sleep(delayTime)

    elif keyPress == 't':
        msg = "Throwing.\n"
        cmdLog.insert(0.0, msg)
        throwerRelativeRPM = 1050

        throw.Baller(robotSpeed, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'h':
        msg = "Collecting ball.\n"
        cmdLog.insert(0.0, msg)

        throw.MoveHoldBall(robotSpeed, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'f':
        msg = "Failsafe toggled.\n"
        cmdLog.insert(0.0, msg)

        toggleFailsafe()

        time.sleep(delayTime)

    elif keyPress == 'g':
        msg = "Game logic toggled.\n"
        toggleGameLogic()

        time.sleep(delayTime)
    
    elif keyPress == 'x':
        closeAll()

    #elif keyPress.isdigit(): # could use this to assign robot motor speed values manually with number keys
    #    if int(keyPress) in servo_range:
    #        msg = keyPress
    #        cmdLog.insert(0.0, msg)
    #        servo(int(keyPress)*14)
    #        time.sleep(0.05)

################################################################################################################################################
################################################################################################################################################

failsafe = 0

def toggleFailsafe():
    
    global failsafe
    
    if failsafeButton.config('relief')[-1] == 'sunken':
        failsafeButton.config(relief="raised")
        failsafe = 0
    else:
        failsafeButton.config(relief="sunken", background="red")
        failsafe = 1

gameLogicStart = False

def toggleGameLogic():
    
    global gameLogicStart

    if gameLogicButton.config('relief')[-1] == 'sunken':
        gameLogicButton.config(relief="raised")
        gameLogicStart = False # game logic off, key inputs enabled
        disableButtonsToggle()
    else:
        gameLogicButton.config(relief="sunken")
        gameLogicStart = True # game logic on, disables other key inputs
        disableButtonsToggle()

targetColor = "magenta" # using magenta because pink is not available here: https://anzeljg.github.io/rin2/book2/2405/docs/tkinter/colors.html
# 'pink' basket as default basket (setTarget variable in other modules)

def getGameState():
    global targetColor, gameLogicStart
    return targetColor, gameLogicStart

def toggleTargetBasket():
    
    global targetColor

    if targetBasketButton.config('relief')[-1] == 'sunken':
        targetBasketButton.config(relief="raised")
        targetColor = "magenta" # target pink basket
        targetBasketButton["text"] = targetColor
        targetBasketButton["background"] = targetColor
        basketColor(targetColor)
    else:
        targetBasketButton.config(relief="sunken")
        targetColor = "blue" # target blue basket
        targetBasketButton["text"] = targetColor
        targetBasketButton["background"] = targetColor
        basketColor(targetColor)


def disableButtonsToggle():
    if failsafeButton["state"] == "normal" and gameLogicStart == True:
        failsafeButton["state"] = "disabled"
        targetBasketButton["state"] = "disabled"
        gameLogicButton["text"] = "Game Logic Enabled"
        gameLogicButton["background"] = "green"
    else:
        failsafeButton["state"] = "enabled"
        targetBasketButton["state"] = "enabled"
        gameLogicButton["text"] = "Game Logic Disabled"
        gameLogicButton["background"] = "red"


def basketColor(targetColor): # https://anzeljg.github.io/rin2/book2/2405/docs/tkinter/create_rectangle.html
    basketCanvas.create_rectangle(20, 20, 80, 80, width=0, fill=targetColor)
    cmdLog.insert(0.0, targetColor+"\n")

# Function for closing window
def closeAll():
    msg = "Shutting down.\n"
    cmdLog.insert(0.0, msg)
    print(msg)
    toggleGameLogic()
    gameLogicStart = False
    move.allStop()
    time.sleep(0.05)
    cv2.destroyAllWindows()
    rootWindow.destroy()


################################################################################################################################################
################################################################################################################################################

# src inspiration: https://robotic-controls.com/learn/python-guis/basics-tkinter-gui

# Creating the tkinter window
rootWindow = Tk()
rootWindow.wm_title("B T S Control Panel")

if gameLogicStart == False: # if game logic not toggled on, listen for manual commands
    rootWindow.bind_all('<Key>', keyInput) # this calls the key input loop for cmds


pressedKeyCheck = event.keysym.lower()
if pressedKeyCheck == 'x': # additional safety feature that listens for key x even when game logic is running
    closeAll() # if x pressed, emergency stop

rootWindow.config(background="#35a0de")
instructions = "\n w = forward \n d = right \n a = left \n s = back \n q = spin left \n e = spin right \n t = thrower \n space = stop \n f = toggle failsafe \n x = kill programme \n"

################################################################################################################################################
################################################################################################################################################

#Left Frame and its contents
leftFrame = Frame(rootWindow, width=200, height = 600)
leftFrame.grid(row=0, column=0, padx=10, pady=2)

Label(leftFrame, text="Operating instructions").grid(row=0, column=0, padx=10, pady=2)

instructionsLabel = Label(leftFrame, text=instructions, font='bold')
instructionsLabel.grid(row=1, column=0, padx=10, pady=2)

basketCanvas = Canvas(leftFrame, width=100, height=100, bg='white')
basketCanvas.grid(row=2, column=0, padx=10, pady=2)

################################################################################################################################################
################################################################################################################################################

#Right Frame and its contents
rightFrame = Frame(rootWindow, width=200, height = 600)
rightFrame.grid(row=0, column=1, padx=10, pady=2)

cmdLog = Text(rightFrame, width = 30, height = 20, takefocus=0)
cmdLog.grid(row=2, column=0, padx=10, pady=2)

btnFrame = Frame(rightFrame, width=200, height = 200)
btnFrame.grid(row=1, column=0, padx=10, pady=2)

exitBtn = Button(btnFrame, text="Exit", command=closeAll, font='bold')
exitBtn.grid(row=1, column=0, padx=10, pady=2)

failsafeButton = Button(btnFrame, text="Toggle Failsafe", relief="raised", command=toggleFailsafe, font='bold')
failsafeButton.grid(row=1, column=1, padx=10, pady=2)

gameLogicButton = Button(btnFrame, text="Game Logic Disabled", relief="raised", command=toggleGameLogic, font='bold')
gameLogicButton.grid(row=1, column=2, padx=10, pady=2)

targetBasketButton = Button(btnFrame, text="magenta", relief="raised", command=toggleTargetBasket, font='bold')
#targetBasketButton = Button(btnFrame, text="Toggle Target", relief="raised", command=toggleTargetBasket, font='bold')
targetBasketButton.grid(row=1, column=2, padx=10, pady=2)

################################################################################################################################################
################################################################################################################################################

rootWindow.mainloop()

################################################################################################################################################
################################################################################################################################################
