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

def failsafe_check():
	disable_failsafe_input = int(input("Failsafe speed [integer: 0 or 1]: "))
	if disable_failsafe_input == 1:
		print("Failsafe disabled!")
		disable_failsafe = 1
	elif disable_failsafe_input == 0:
		disable_failsafe = 0
		print("Failsafe enabled!")
	else:
		print("Invalid, keeping failsafe on.")
		disable_failsafe = 0
	return disable_failsafe

def motor_speed_val():
	check = input("Turn on motors ==> [y] or hit enter to turn all motors off. ")
	if check == 'y':
		motor_speed_1_input = int(input("Motor 1 speed [integer in range 0 to 100]: "))
		print("Motor speed 1 = ", motor_speed_1_input)
		motor_speed_2_input = int(input("Motor 2 speed [integer in range 0 to 100]: "))
		print("Motor speed 2 = ", motor_speed_2_input)
		motor_speed_3_input = int(input("Motor 3 speed [integer in range 0 to 100]: "))
		print("Motor speed 3 = ", motor_speed_3_input)
		thrower_speed_input = int(input("Thrower motor speed [integer in range 0 to 1800]: "))
		print("Thrower motor speed = ", thrower_speed_input)
	else:
		motor_speed_1_input = 0
		motor_speed_2_input = 0
		motor_speed_3_input = 0
		thrower_speed_input = 0

	speeds = [motor_speed_1_input, motor_speed_2_input, motor_speed_3_input, thrower_speed_input]
	return speeds

count = 0

#disable_failsafe = 0
disable_failsafe = failsafe_check()

#speed1 = 0
#speed2 = 0
#speed3 = 0
#thrower_speed = 1500
#thrower_speed = 0
#thrower_speed = int(input("input thrower speed [integer!]: "))
#speeds = [speed1, speed2, speed3, thrower_speed]

speeds = motor_speed_val()
#print(speeds)

ser = None
try:
	port='/dev/ttyACM1'
	ser = serial.Serial(port, baudrate=115200, timeout=3)
	#while True:
	print('Loop started!')
	while count < 1000: # resulting motion at least 1m?
		motor_data = send_ms(ser,speeds)
		#send_ms(ser,speeds)
		#print(motor_data)
		#sleep(1.05)
		count += 1
		#print('loop iteration =  ',count)
	
	print('Loop finished!')
except Exception as e:
	print(e)
if ser != None:
	ser.close()
