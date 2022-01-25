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

# using magenta because pink is not available here: https://anzeljg.github.io/rin2/book2/2405/docs/tkinter/colors.html
# 'pink' basket as default basket (setTarget variable in other modules)

class BTSControlPanel:
    def __init__(self, shared_data, o_ws, o_game):
        self.shared_data = shared_data
        self.o_ws = o_ws
        self.o_game = o_game
        self.shared_data["gameLogicStart"] = False
        self.failsafe = 0
        self.target_color = shared_data["targetColor"]
        self.running = True
    
    def keyInput(self,event):
        #Log was only input but now new field for ws connection exists. 
        if(self.rootWindow.focus_get() == self.cmdLog or self.rootWindow.focus_get()==self.rootWindow):
            delayTime = 0.005
            throwerRelativeRPM = 0
            robotSpeed = 60

            keyPress = event.keysym.lower()
            if self.shared_data["gameLogicStart"] == False:
                
                if keyPress == 'w':
                    msg = "Moving forward.\n"
                    self.cmdLog.insert(0.0, msg)
                    robotDirectionAngle = 90
                    robotAngularVelocity = 0
                        
                    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, self.failsafe)
                    time.sleep(delayTime)

                elif keyPress == 'd':
                    msg = "Moving right.\n"
                    self.cmdLog.insert(0.0, msg)
                    robotDirectionAngle = 0
                    robotAngularVelocity = 0
                        
                    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, self.failsafe)
                    time.sleep(delayTime)

                elif keyPress == 's':
                    msg = "Moving back.\n"
                    self.cmdLog.insert(0.0, msg)
                    robotDirectionAngle = 270
                    robotAngularVelocity = 0
                        
                    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, self.failsafe)
                    time.sleep(delayTime)

                elif keyPress == 'a':
                    msg = "Moving left.\n"
                    self.cmdLog.insert(0.0, msg)
                    robotDirectionAngle = 180
                    robotAngularVelocity = 0

                    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, self.failsafe)
                    time.sleep(delayTime)

                elif keyPress == 'e':
                    msg = "Spinning right.\n"
                    self.cmdLog.insert(0.0, msg)
                    robotSpeed = 0
                    robotDirectionAngle = 0
                    robotAngularVelocity = -50
                        
                    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, self.failsafe)
                    time.sleep(delayTime)

                elif keyPress == 'q':
                    msg = "Spinning left.\n"
                    self.cmdLog.insert(0.0, msg)
                    robotSpeed = 0
                    robotDirectionAngle = 0
                    robotAngularVelocity = 50
                        
                    move.omniDirect(robotSpeed, robotDirectionAngle, robotAngularVelocity, throwerRelativeRPM, self.failsafe)
                    time.sleep(delayTime)

                elif keyPress == 'space':
                    msg = "Stopped.\n"
                    self.cmdLog.insert(0.0, msg)

                    move.allStop()
                    time.sleep(delayTime)

                elif keyPress == 't':
                    msg = "Throwing.\n"
                    self.cmdLog.insert(0.0, msg)
                    throwerRelativeRPM = 1050

                    move.Baller(robotSpeed, throwerRelativeRPM, self.failsafe)
                    time.sleep(delayTime)

                elif keyPress == 'h':
                    msg = "Collecting ball.\n"
                    self.cmdLog.insert(0.0, msg)

                    move.MoveHoldBall(robotSpeed, self.failsafe)
                    time.sleep(delayTime)
                elif keyPress == 'f':
                    msg = "Failsafe toggled.\n"
                    self.cmdLog.insert(0.0, msg)

                    self.toggleFailsafe()
                    time.sleep(delayTime)

                elif keyPress == 'b':
                    self.toggleTargetBasket()
                    time.sleep(delayTime)

                elif keyPress == 'g':
                    self.toggleGameLogic()
                    time.sleep(delayTime)
                
                elif keyPress == 'x':
                    self.closeAll()

                #elif keyPress.isdigit(): # could use this to assign robot motor speed values manually with number keys
                #    if int(keyPress) in servo_range:
                #        msg = keyPress
                #        self.cmdLog.insert(0.0, msg)
                #        servo(int(keyPress)*14)
                #        time.sleep(0.05)
            
            elif self.shared_data["gameLogicStart"] == True: # additional safety feature that listens for key x even when game logic is running                
                if keyPress == 'x':
                    self.closeAll()
                
                elif keyPress == 'g':
                    self.toggleGameLogic()
                    time.sleep(delayTime)
                elif keyPress == 'c':
                    msg = "Stop Running Game.\n"
                    self.cmdLog.insert(0.0, msg)
                    self.shared_data['gameThreadUp'] = False  
                    
        

    ################################################################################################################################################
    ################################################################################################################################################

    def toggleFailsafe(self):
        
        if self.failsafeButton.config('relief')[-1] == 'sunken':
            self.failsafeButton.config(relief="raised")
            self.failsafe = 0
        else:
            self.failsafeButton.config(relief="sunken")
            self.failsafe = 1

    def toggleGameLogic(self):
        

        if self.gameLogicButton.config('relief')[-1] == 'sunken':
            self.gameLogicButton.config(relief="raised")
            self.shared_data["gameLogicStart"] = False # game logic off, key inputs enabled
            msg = "Game Logic Disabled\n"
            move.allStop()
            self.cmdLog.insert(0.0, msg)
            cv2.destroyAllWindows()
            self.disableButtonsToggle()

        else:
            self.gameLogicButton.config(relief="sunken")
            self.shared_data["gameLogicStart"] = True # game logic on, disables other key inputs
            msg = "Game Logic Enabled\n"
            self.cmdLog.insert(0.0, msg)
            self.disableButtonsToggle()

        #print(f"\nGame Logic Start variable from GUI: {shared_data["gameLogicStart"]}\n")\

    def runGameLogic(self):
        if self.shared_data["gameLogicStart"] == True and self.shared_data["gameThreadUp"] == False:
            msg = "Game Logic Running.\n"
            self.cmdLog.insert(0.0, msg)
            msg = "Good luck little robit!.\n"
            self.cmdLog.insert(0.0, msg)
            msg = "Press c to exit!\n"
            self.cmdLog.insert(0.0, msg)
            print(msg)
            #chokeGameButton["state"] = "disabled"
            #Common memory for gameLogic
            self.shared_data['gameState'] = 0
            game_thread = threading.Thread(target = self.o_game.infiniteGameLoop, args = ())
            game_thread.start()            
            #game.runGameLogic(self.shared_data, self.shared_data["gameLogicStart"])
        elif  self.shared_data["gameThreadUp"] == True:
            msg = "Game Logic Already Running.\n"
            self.cmdLog.insert(0.0, msg)


    def toggleTargetBasket(self):
        if self.targetBasketButton.config('relief')[-1] == 'sunken':
            self.targetBasketButton.config(relief="raised")
            self.target_color = "magenta" # target pink basket
            self.shared_data['targetColor'] = "magenta"
            msg = "Target: Magenta Basket\n"
            self.cmdLog.insert(0.0, msg)
            self.basketColorUpdate()            
        else:
            self.targetBasketButton.config(relief="sunken")
            self.target_color = "blue" # target blue basket
            self.shared_data['targetColor'] = "blue"
            msg = "Target: Blue Basket\n"
            self.cmdLog.insert(0.0, msg)
            self.basketColorUpdate()
            
    #Creates or ends connection with thread. TODO am not sure if name is changed to disconnected on disconection from server 
    def toggleWSButton(self):  
        if(self.startWSButton['text'] == "Connect"):           
            link = self.wsLog.get(0.0,'end-1c')
            print(link)
            print("Toogle client thread")
            self.ws_thread = threading.Thread(target = self.o_ws.wsConnection, args = [link])
            self.ws_thread.start()
            self.startWSButton.config(text="Disconnect")
        else:
            print("End client thread")
            #ws_thread.stop()
            self.shared_data['wsStop'] = True
            self.ws_thread.join()
            print("Done")
            self.startWSButton.config(text="Connect")

    def basketColorUpdate(self): # https://anzeljg.github.io/rin2/book2/2405/docs/tkinter/create_rectangle.html
        self.basketCanvas.create_rectangle(20, 20, 80, 80, width=0, fill=self.target_color)

    def disableButtonsToggle(self):

        if self.shared_data["gameLogicStart"] == True:
            self.failsafeButton.config(relief="raised")
            self.failsafe = 0
            self.failsafeButton["state"] = "disabled"
            self.targetBasketButton["state"] = "disabled"
            #chokeGameButton["state"] = "enabled"

        elif self.shared_data["gameLogicStart"] == False:
            self.failsafeButton["state"] = "active"
            self.targetBasketButton["state"] = "active"
            #chokeGameButton["state"] = "disabled"

    # Function for closing window
    # May miss disconect for ws
    #TODO add correct thread kill!!!!
    def closeAll(self):
        msg = "Shutting down.\n"
        self.cmdLog.insert(0.0, msg)
        print(msg)
        self.shared_data["gameLogicStart"] = False
        #threading.Thread(self.o_game.infiniteCameraProcessing()).start()
        #game.runGameLogic(self.shared_data, self.shared_data["gameLogicStart"])
        self.shared_data['wsStop'] = True    
        self.shared_data['camThreadUp'] = False
        self.shared_data['gameThreadUp'] = False        
        self.running = False


    ################################################################################################################################################
    ################################################################################################################################################

    # src inspiration: https://robotic-controls.com/learn/python-guis/basics-tkinter-gui


    # Creating the tkinter window
    def startPanel(self):
        self.rootWindow = Tk()
        self.rootWindow.wm_title("B T S Control Panel")

        self.rootWindow.bind_all('<Key>', self.keyInput) # this calls the key input loop for cmds

        self.rootWindow.config(background="#35a0de")

        ################################################################################################################################################
        ################################################################################################################################################
        
        instructions = "\n Select target basket then click toggle game logic. \n Press Run game logic.\nw = forward \n d = right \n a = left \n s = back \n q = spin left \n e = spin right \n t = thrower \n space = stop \n f = toggle failsafe \n x = kill programme \n"

        #Left Frame and its contents
        leftFrame = Frame(self.rootWindow, width=200, height = 600)
        leftFrame.grid(row=0, column=0, padx=10, pady=2)

        Label(leftFrame, text="Operating instructions").grid(row=0, column=0, padx=10, pady=2)

        instructionsLabel = Label(leftFrame, text=instructions)
        instructionsLabel.grid(row=1, column=0, padx=10, pady=2)

        self.basketCanvas = Canvas(leftFrame, width=100, height=100, bg='white')
        self.basketCanvas.grid(row=2, column=0, padx=10, pady=2)

        ################################################################################################################################################
        ################################################################################################################################################

        #Right Frame and its contents
        rightFrame = Frame(self.rootWindow, width=200, height = 600)
        rightFrame.grid(row=0, column=1, padx=10, pady=2)
        textFrame = Frame(rightFrame, width=200, height = 300)
        textFrame.grid(row=2, column=0, padx=10, pady=2)
        
        self.cmdLog = Text(textFrame, width = 30, height = 20, takefocus=0)
        self.cmdLog.grid(row=1, column=0, padx=10, pady=2)

        btnFrame = Frame(rightFrame, width=200, height = 200)
        btnFrame.grid(row=1, column=0, padx=10, pady=2)

        exitBtn = Button(btnFrame, text="Exit", command=self.closeAll)
        exitBtn.grid(row=1, column=0, padx=10, pady=2)

        self.failsafeButton = Button(btnFrame, text="Toggle Failsafe", command=self.toggleFailsafe)
        self.failsafeButton.grid(row=1, column=1, padx=10, pady=2)

        self.gameLogicButton = Button(btnFrame, text="Game Logic Toggle", command=self.toggleGameLogic)
        self.gameLogicButton.grid(row=1, column=2, padx=10, pady=2)

        self.targetBasketButton = Button(btnFrame, text="Change Basket", command=self.toggleTargetBasket)
        self.targetBasketButton.grid(row=1, column=3, padx=10, pady=2)

        chokeGameButton = Button(btnFrame, text="Run Game Logic", command=self.runGameLogic)
        chokeGameButton.grid(row=1, column=4, padx=10, pady=2)
        
        self.wsLog = Text(textFrame, width = 20, height = 1, takefocus=1)
        self.wsLog.grid(row=1, column=1, padx=10, pady=2, sticky=N)
        self.startWSButton = Button(textFrame, text="Connect", command=self.toggleWSButton)
        self.startWSButton.grid(row=1, column=1, padx=10, pady=30, sticky=N)

        self.basketColorUpdate()

        ################################################################################################################################################
        ################################################################################################################################################
        while self.running:
            time.sleep(0.01)            
            if (self.shared_data['wsThreadUp'] == True and self.startWSButton['text'] != "Disconnect"):
                self.startWSButton['text'] = "Disconnect"
            elif (self.shared_data['wsThreadUp'] == False and self.startWSButton['text'] != "Connect"):
                self.startWSButton['text'] = "Connect"
            if(self.shared_data['targetColor'] != self.target_color):
                self.toggleTargetBasket()
            if(self.shared_data['gameState'] == 0 and self.shared_data["gameLogicStart"] == False):
                self.toggleGameLogic()
            elif(self.shared_data['gameState'] == 0 and self.shared_data["gameThreadUp"] == False):
                self.runGameLogic()
            elif(self.shared_data['gameState'] == -1 and self.shared_data["gameThreadUp"] == True):
                self.shared_data["gameThreadUp"] = False
            self.rootWindow.update_idletasks()
            self.rootWindow.update()
            
        move.allStop()
        time.sleep(0.05)
        cv2.destroyAllWindows()
        self.rootWindow.destroy()
