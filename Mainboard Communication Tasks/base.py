import serial
from time import sleep
import serial.tools.list_ports
import motionModule as mt
mainBoardPort='/dev/ttyACM1'

#count = 0
#ser = None
'''
ports= serial.tools.list_ports.comports()
print([port.name for port in ports])
'''
try:	 
    mtCntrl = mt.MotionControll(mainBoardPort)
    mtCntrl.motor_speed_val_input() #default values are inside
    mtCntrl.failsafe_check_input() #default values are inside
	#while True:
	#ser = self.ser
    print('Loop started!')
	
    count = 0
    while count < 1000: # resulting motion at least 1m?
        mtCntrl.send_motorspeeds()
        #send_ms(ser,speeds)
        #print(mtCntrl.res)
        #sleep(1.05)
        count += 1
        #print('loop iteration =  ',count)
    print('Loop finished!') 
	
except Exception as e:
	print(e)
