import serial
import struct
import math
from time import sleep

def send_ms(ser,speeds): #unpack motorspeeds
    return send_motorspeeds(ser,speeds[0],speeds[1],speeds[2],speeds[3]) #returns motor data


def send_motorspeeds(ser,m1 = 0,m2 = 0,m3 = 0,thrower = 0): # send speeds to motors and return data
    write_this = struct.pack('<hhhHBH', m1, m2, m3, thrower, disable_failsafe, 0xAAAA)
    #print(write_this)
    ser.write(write_this)
    size = struct.calcsize('<hhhH')
    buffer_size = ser.read(size)
    #buffer_size = ser.readline()
    values = struct.unpack('<hhhH',buffer_size)
    return values #returns motor data

count = 0

disable_failsafe = 0
speed1 = 15
speed2 = 5
speed3 = -15
thrower_speed = 0

speeds = [speed1, speed2, speed3, thrower_speed]

ser = None
try:
	port='/dev/ttyACM1'
	ser = serial.Serial(port, baudrate=115200, timeout=3)
	#while True:
	print('Loop started!')
	while count < 10000: # resulting motion at least 1m?
		motor_data = send_ms(ser,speeds)
		#send_ms(ser,speeds)
		print(motor_data)
		sleep(1.05)
		count += 1
		#print('loop iteration =  ',count)
	
	print('Loop finished!')
except Exception as e:
	print(e)
if ser != None:
	ser.close()
