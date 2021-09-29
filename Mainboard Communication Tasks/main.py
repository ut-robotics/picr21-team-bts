import serial
import struct
import time

ser = serial.Serial('COM9')
print(ser.name)
ser.write(struct.pack('<hhhHBH', 0, 0, 0, 0, 0, 0xAAAA))

#time.sleep(0.2)

while ser.in_waiting > 0:
    ser.read()
    #received_byte = ser.read()
    #print(received_byte)


#struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)

ser.close()