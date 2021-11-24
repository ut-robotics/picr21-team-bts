import rc_inator as gui
import threading

'''
This is the Robot Manager module for the B T S test robot.
'''
print("robit_manager_inator done it's job!\n Alright!\n")
threading.Thread(gui.BTSControlPanel()).start() # start tkinter gui fcn
