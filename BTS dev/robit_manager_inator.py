import rc_inator as gui
import threading
import ws_client
import game_logician_inator as game

'''
This is the Robot Manager module for the B T S test robot.
'''
print("robit_manager_inator done it's job!\n Alright!\n")
shared_data = dict(
    gameState = -1, #0-Idle -1 - Stop
    targetColor =  "magenta",
    gameLogicStart = False,
    wsStop = False,
    gameThreadUp = False,
    wsThreadUp = False,
    camThreadUp = False,
    name = "BTS"
)
o_ws = ws_client.wsConnectionClass(shared_data)
o_game = game.GameLogic(shared_data)
o_gui = gui.BTSControlPanel(shared_data,o_ws,o_game)
cam_thread = threading.Thread(target = o_game.infiniteCameraProcessing, args = ())
cam_thread.start()
threading.Thread(o_gui.startPanel()).start() # start tkinter gui fcn
#threading.Thread(o_game.infiniteCameraProcessing()).start()


