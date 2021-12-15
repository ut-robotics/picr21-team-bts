import maneuver_inator as move
#import thrower_inator as throw
import game_logician_inator as game
import cv2
import numpy as np
import time
from tkinter import *
from mttkinter import mtTkinter
import threading
import ws_client

'''
This is the Graphical User Interface module for the B T S test robot.

When run, a window is created featuring instructions, buttons and a text log of command inputs.

'''
global gameLogicStart, failsafe, targetColor, ws_thread, shared_data
shared_data = dict(
    gameState = -1, #0-Idle -1 - Stop
    targetColor =  "magenta",
    wsStop = False
)

gameLogicStart = False
ws_thread_stop = False
failsafe = 0
targetColor = "magenta" # using magenta because pink is not available here: https://anzeljg.github.io/rin2/book2/2405/docs/tkinter/colors.html
# 'pink' basket as default basket (setTarget variable in other modules)
ws_thread = threading.Thread(target = ws_client.wsConnection, args = (shared_data))
def BTSControlPanel():
    global gameLogicStart, failsafe, targetColor, ws_thread, ws_thread_stop, shared_data
    
    def keyInput(event):
        #e2 = Text(rootWindow, text = "Log")   
        if(rootWindow.focus_get() == cmdLog or rootWindow.focus_get()==rootWindow):
            delayTime = 0.005
            throwerRelativeRPM = 0
            robotSpeed = 20

            keyPress = event.keysym.lower()
            if gameLogicStart == False:
                
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
                    robotAngularVelocity = -50
                        
                    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, failsafe)
                    time.sleep(delayTime)

                elif keyPress == 'q':
                    msg = "Spinning left.\n"
                    cmdLog.insert(0.0, msg)
                    robotSpeed = 0
                    robotDirectionAngle = 0
                    robotAngularVelocity = 50
                        
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

                    move.Baller(robotSpeed, throwerRelativeRPM, failsafe)
                    time.sleep(delayTime)

                elif keyPress == 'h':
                    msg = "Collecting ball.\n"
                    cmdLog.insert(0.0, msg)

                    move.MoveHoldBall(robotSpeed, failsafe)
                    time.sleep(delayTime)
                elif keyPress == 'f':
                    msg = "Failsafe toggled.\n"
                    cmdLog.insert(0.0, msg)

                    toggleFailsafe()
                    time.sleep(delayTime)

                elif keyPress == 'b':
                    toggleTargetBasket()
                    time.sleep(delayTime)

                elif keyPress == 'g':
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
            
            elif gameLogicStart == True: # additional safety feature that listens for key x even when game logic is running
                
                if keyPress == 'x':
                    closeAll()
                
                elif keyPress == 'g':
                    toggleGameLogic()
                    time.sleep(delayTime)
        

    ################################################################################################################################################
    ################################################################################################################################################

    def toggleFailsafe():
        
        global failsafe
        
        if failsafeButton.config('relief')[-1] == 'sunken':
            failsafeButton.config(relief="raised")
            failsafe = 0
        else:
            failsafeButton.config(relief="sunken")
            failsafe = 1

    def toggleGameLogic():
        
        global gameLogicStart

        if gameLogicButton.config('relief')[-1] == 'sunken':
            gameLogicButton.config(relief="raised")
            gameLogicStart = False # game logic off, key inputs enabled
            msg = "Game Logic Disabled\n"
            move.allStop()
            cmdLog.insert(0.0, msg)
            cv2.destroyAllWindows()
            disableButtonsToggle()

        else:
            gameLogicButton.config(relief="sunken")
            gameLogicStart = True # game logic on, disables other key inputs
            msg = "Game Logic Enabled\n"
            cmdLog.insert(0.0, msg)
            disableButtonsToggle()

        #print(f"\nGame Logic Start variable from GUI: {gameLogicStart}\n")\

    def runGameLogic():
        global gameLogicStart, shared_data
        if gameLogicStart == True:
            msg = "Game Logic Running.\n"
            cmdLog.insert(0.0, msg)
            msg = "Good luck little robit!.\n"
            cmdLog.insert(0.0, msg)
            msg = "Press q to exit!\n"
            cmdLog.insert(0.0, msg)
            print(msg)
            #chokeGameButton["state"] = "disabled"
            shared_data['gameState'] = 0
            game.runGameLogic(shared_data, gameLogicStart)


    def toggleTargetBasket():
        global targetColor

        if targetBasketButton.config('relief')[-1] == 'sunken':
            targetBasketButton.config(relief="raised")
            targetColor = "magenta" # target pink basket
            shared_data['targetColor'] = "magenta"
            msg = "Target: Magenta Basket\n"
            cmdLog.insert(0.0, msg)
            basketColor(targetColor)            
        else:
            targetBasketButton.config(relief="sunken")
            targetColor = "blue" # target blue basket
            shared_data['targetColor'] = "blue"
            msg = "Target: Blue Basket\n"
            cmdLog.insert(0.0, msg)
            basketColor(targetColor)
    
    def toggleWSButton():
        global targetColor, ws_thread    
        if(startWSButton['text'] == "Connect"):
            print(wsLog.get(0.0,'end-1c'))
            print("Toogle client thread")
            link = wsLog.get(0.0,'end-1c')
            ws_thread = threading.Thread(target = ws_client.wsConnection, args = (shared_data,link))
            ws_thread.start()
            startWSButton.config(text="Disconnect")
        else:
            print("End client thread")
            #ws_thread.stop()
            shared_data['wsStop'] = True
            ws_thread.join()
            print("Done")
            startWSButton.config(text="Connect")

    def basketColor(targetColor): # https://anzeljg.github.io/rin2/book2/2405/docs/tkinter/create_rectangle.html
        basketCanvas.create_rectangle(20, 20, 80, 80, width=0, fill=targetColor)

    def disableButtonsToggle():
        
        global gameLogicStart, failsafe

        if gameLogicStart == True:
            failsafeButton.config(relief="raised")
            failsafe = 0
            failsafeButton["state"] = "disabled"
            targetBasketButton["state"] = "disabled"
            #chokeGameButton["state"] = "enabled"

        elif gameLogicStart == False:
            failsafeButton["state"] = "active"
            targetBasketButton["state"] = "active"
            #chokeGameButton["state"] = "disabled"

    # Function for closing window
    def closeAll():
        msg = "Shutting down.\n"
        cmdLog.insert(0.0, msg)
        print(msg)
        gameLogicStart = False
        game.runGameLogic(shared_data, gameLogicStart)
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

    rootWindow.bind_all('<Key>', keyInput) # this calls the key input loop for cmds

    rootWindow.config(background="#35a0de")

    ################################################################################################################################################
    ################################################################################################################################################
    
    instructions = "\n Select target basket then click toggle game logic. \n Press Run game logic.\nw = forward \n d = right \n a = left \n s = back \n q = spin left \n e = spin right \n t = thrower \n space = stop \n f = toggle failsafe \n x = kill programme \n"

    #Left Frame and its contents
    leftFrame = Frame(rootWindow, width=200, height = 600)
    leftFrame.grid(row=0, column=0, padx=10, pady=2)

    Label(leftFrame, text="Operating instructions").grid(row=0, column=0, padx=10, pady=2)

    instructionsLabel = Label(leftFrame, text=instructions)
    instructionsLabel.grid(row=1, column=0, padx=10, pady=2)

    basketCanvas = Canvas(leftFrame, width=100, height=100, bg='white')
    basketCanvas.grid(row=2, column=0, padx=10, pady=2)

    ################################################################################################################################################
    ################################################################################################################################################

    #Right Frame and its contents
    rightFrame = Frame(rootWindow, width=200, height = 600)
    rightFrame.grid(row=0, column=1, padx=10, pady=2)
    textFrame = Frame(rightFrame, width=200, height = 300)
    textFrame.grid(row=2, column=0, padx=10, pady=2)
    
    cmdLog = Text(textFrame, width = 30, height = 20, takefocus=0)
    cmdLog.grid(row=1, column=0, padx=10, pady=2)

    btnFrame = Frame(rightFrame, width=200, height = 200)
    btnFrame.grid(row=1, column=0, padx=10, pady=2)

    exitBtn = Button(btnFrame, text="Exit", command=closeAll)
    exitBtn.grid(row=1, column=0, padx=10, pady=2)

    failsafeButton = Button(btnFrame, text="Toggle Failsafe", command=toggleFailsafe)
    failsafeButton.grid(row=1, column=1, padx=10, pady=2)

    gameLogicButton = Button(btnFrame, text="Game Logic Toggle", command=toggleGameLogic)
    gameLogicButton.grid(row=1, column=2, padx=10, pady=2)

    targetBasketButton = Button(btnFrame, text="Change Basket", command=toggleTargetBasket)
    targetBasketButton.grid(row=1, column=3, padx=10, pady=2)

    chokeGameButton = Button(btnFrame, text="Run Game Logic", command=runGameLogic)
    chokeGameButton.grid(row=1, column=4, padx=10, pady=2)
    
    wsLog = Text(textFrame, width = 20, height = 1, takefocus=1)
    wsLog.grid(row=1, column=1, padx=10, pady=2, sticky=N)
    startWSButton = Button(textFrame, text="Connect", command=toggleWSButton)
    startWSButton.grid(row=1, column=1, padx=10, pady=30, sticky=N)

    basketColor(targetColor)

    ################################################################################################################################################
    ################################################################################################################################################
    global shared_data
    while True:
        #print(shared_data)
        time.sleep(0.01)
        if(shared_data['targetColor'] != targetColor):
            toggleTargetBasket()
        if(shared_data['gameState'] == 0 and gameLogicStart == False):
            toggleGameLogic()
        if(shared_data['gameState'] == 0):
            #print("START-----")
            runGameLogic()
        rootWindow.update_idletasks()
        rootWindow.update()
