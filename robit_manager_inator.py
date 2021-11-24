import rc_inator as gui
import threading

'''
This is the Robot Manager module for the B T S test robot.
'''
threading.Thread(gui.BTSControlPanel()).start() # start tkinter gui fcn

