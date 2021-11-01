import struct
import math
import serial  
import sys    
from dataclasses import dataclass

@dataclass
class Request:
    m1 = 0
    m2 = 0
    m3 = 0
    thrower = 0
    d_failsafe = 0
    delim = 0xAAAA

class Connection:
    def __init__(self, port):
        self.ser = serial.Serial(port, baudrate = 115200, timeout = 3)

class MotionControll(Connection):        
    def __init__(self, port):
        super().__init__(port)
        self.req = Request()
    def __del__(self):
        if self.ser != None:
            self.ser.close()
    def A(self):
        print("B");
    def setSpeeds(self,ar):
        self.Request.m1 = ar[0]
        self.Request.m2 = ar[1]
        self.Request.m3 = ar[2]
        self.Request.thrower = ar[3]
    #def send_ms(self, speeds = self.motor_speed_val(), failsafe = self.failsafe_check()): #unpack motorspeeds #SAME ORDER
    #    return send_motorspeeds(self.ser, speeds[0],speeds[1],speeds[2],speeds[3], failsafe) #returns motor data
        
    def send_motorspeeds(self):#, m1 = 0, m2 = 0, m3 = 0, thrower = 0, d_failsafe = self.failsafe_check()): # send speeds to motors and return data #SAME ORDER
        write_this = struct.pack('<hhhHBH', self.req.m1, self.req.m2, self.req.m3, self.req.thrower, self.req.d_failsafe, self.req.delim)
        #print(write_this)
        self.ser.write(write_this)
        size = struct.calcsize('<hhhH')
        buffer_size = self.ser.read(size)
        #buffer_size = ser.readline()
        self.res = struct.unpack('<hhhH',buffer_size)
        #return values #returns motor data

    def failsafe_check_input(self): # specify whether you want the failsafe on or off with a 0 or 1
        disable_failsafe_input = input("Failsafe disable? [y]: ")
        if disable_failsafe_input == 'y':
            print("Failsafe disabled!")
            self.req.d_failsafe = 1
        else:
            self.req.d_failsafe = 0
            print("Failsafe enabled!")
        #return disable_failsafe

    def motor_speed_val_input(self):
        check = input("Turn on motors ==> [y] or hit enter to turn all motors off. ")
        if check == 'y':
            self.req.m1 = int(input("Motor 1 speed [integer in range 0 to 100]: "))
            print("Motor speed 1 = ",  self.req.m1)
            self.req.m2 = int(input("Motor 2 speed [integer in range 0 to 100]: "))
            print("Motor speed 2 = ",  self.req.m2)
            self.req.m3 = int(input("Motor 3 speed [integer in range 0 to 100]: "))
            print("Motor speed 3 = ",  self.req.m3)
            self.req.thrower = int(input("Thrower motor speed [integer in range 0 to 1800]: "))
            print("Thrower motor speed = ", self.req.thrower)
        else:
            self.req.m1 = 0
            self.req.m2 = 0
            self.req.m3 = 0
            self.req.thrower = 0
        #speeds = [motor_speed_1_input, motor_speed_2_input, motor_speed_3_input, thrower_speed_input]
        #return speeds


