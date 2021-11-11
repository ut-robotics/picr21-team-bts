import maneuver_inator as move
import thrower_inator as throw
import cv2
import numpy as np
from tkinter import *
import time

########################################################################

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
            
        #move.forward(robotSpeed, failsafe)
        move.omni(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'd':
        msg = "Moving right.\n"
        cmdLog.insert(0.0, msg)
        robotDirectionAngle = 0
        robotAngularVelocity = 0
            
        #move.right(robotSpeed, failsafe)
        move.omni(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 's':
        msg = "Moving back.\n"
        cmdLog.insert(0.0, msg)
        robotDirectionAngle = 270
        robotAngularVelocity = 0
            
        #move.backward(robotSpeed, failsafe)
        move.omni(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'a':
        msg = "Moving left.\n"
        cmdLog.insert(0.0, msg)
        robotDirectionAngle = 180
        robotAngularVelocity = 0

        #move.left(robotSpeed, failsafe)            
        move.omni(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'e':
        msg = "Spinning right.\n"
        cmdLog.insert(0.0, msg)
        robotSpeed = 0
        robotDirectionAngle = 0
        robotAngularVelocity = -30
            
        move.omni(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
        time.sleep(delayTime)

    elif keyPress == 'q':
        msg = "Spinning left.\n"
        cmdLog.insert(0.0, msg)
        robotSpeed = 0
        robotDirectionAngle = 0
        robotAngularVelocity = 30
            
        move.omni(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
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
        toggle()

        time.sleep(delayTime)

    elif keyPress == 'x':
        msg = "Shutting down.\n"
        cmdLog.insert(0.0, msg)
        print(msg)

        move.allStop()
        time.sleep(0.05)
        rootWindow.destroy()

    #elif keyPress.isdigit():
    #    if int(keyPress) in servo_range:
    #        msg = keyPress
    #        cmdLog.insert(0.0, msg)
    #        servo(int(keyPress)*14)
    #        time.sleep(0.05)

########################################################################

failsafe = 0

def toggle():
    global failsafe
    msg = "Failsafe toggled.\n"
    cmdLog.insert(0.0, msg)
    if failsafeButton.config('relief')[-1] == 'sunken':
        failsafeButton.config(relief="raised")
        failsafe = 0

    else:
        failsafeButton.config(relief="sunken")
        failsafe = 1

# Function for closing window
def close():
    rootWindow.destroy()

########################################################################

# Creating the tkinter window
rootWindow = Tk()
rootWindow.bind_all('<Key>', keyInput) # this calls the key input loop for cmds
rootWindow.wm_title("B T S Manual Override")

rootWindow.config(background = "#35a0de")
instructions = "\n w = forward \n d = right \n a = left \n s = back \n q = spin left \n e = spin right \n t = thrower \n space = stop \n f = toggle failsafe \n x = kill programme \n"

#Left Frame and its contents
leftFrame = Frame(rootWindow, width=200, height = 600)
leftFrame.grid(row=0, column=0, padx=10, pady=2)

Label(leftFrame, text="Operating instructions").grid(row=0, column=0, padx=10, pady=2)

instructionsLabel = Label(leftFrame, text=instructions, font='bold')
instructionsLabel.grid(row=1, column=0, padx=10, pady=2)

#Right Frame and its contents
rightFrame = Frame(rootWindow, width=200, height = 600)
rightFrame.grid(row=0, column=1, padx=10, pady=2)

cmdLog = Text(rightFrame, width = 30, height = 10, takefocus=0)
cmdLog.grid(row=2, column=0, padx=10, pady=2)

btnFrame = Frame(rightFrame, width=200, height = 200)
btnFrame.grid(row=1, column=0, padx=10, pady=2)

exitBtn = Button(btnFrame, text="Exit", command=close, font='bold')
exitBtn.grid(row=1, column=0, padx=10, pady=2)

failsafeButton = Button(btnFrame, text="Toggle Failsafe", relief="raised", command=toggle, background="red", font='bold')
failsafeButton.grid(row=1, column=1, padx=10, pady=2)

rootWindow.mainloop()
########################################################################