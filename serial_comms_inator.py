import struct
import serial
import serial.tools.list_ports


'''
This is the Serial Communication Management module for the B T S test robot.
'''

################################################################################################################################################
################################################################################################################################################

# class instance that handles mainboard communication neatly
class MainboardComms():

    def __init__(self): # using class constructor to iterate thru all ports and establish connection on correct port or raise error
        
        self.dataSize = struct.calcsize('<hhhH') # preset for send/receive data size
        self.testData = struct.pack('<hhhHBH', 0, 0, 0, 0, 0, 0xAAAA)
        self.ports = serial.tools.list_ports.comports()
        
        for port in self.ports:
            try:
                self.testPort = "/dev/" + port.name # serial.Serial needs specific notation to open a serial com port
                print("Testing on port: ", self.testPort)
                self.testSer = serial.Serial(self.testPort, baudrate = 115200, timeout = 2) # test serial port with provided port from list of ports available
                self.testSer.write(self.testData)
                self.receivedTestData = self.testSer.read(self.dataSize)
                self.testData = struct.unpack('<hhhH', self.receivedTestData)
                self.testSer.close()
                self.thePort = self.testPort
            except Exception as e:
                #print(f"This port {self.testPort} is not the port...\n Because: ", e)
                continue
            finally:
                self.ser = serial.Serial(self.thePort, baudrate = 115200, timeout = 2)
        # communication established, keep channel open
        print("Mainboard serial communication established successfully on port: ", self.thePort)

    def SendCmd2Mbd(self, speed1, speed2, speed3, thrower_speed, disable_failsafe):
    
        try: # despite already having checked if the port is accessible, use try in case of HW failures
    
            data = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)
    
            self.ser.write(data)
    
            receivedData = self.ser.read(self.dataSize)
    
            data = struct.unpack('<hhhH', receivedData)
    
        except Exception as e:
            raise e() # if HW failure happens, will likely raise this error at this point

################################################################################################################################################
################################################################################################################################################
