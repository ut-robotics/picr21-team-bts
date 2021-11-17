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
                self.ser = serial.Serial(self.testPort, baudrate = 115200, timeout = 2) # test serial port with provided port from list of ports available
                self.ser.write(self.testData)
                self.receivedTestData = self.ser.read(self.dataSize)
                self.testData = struct.unpack('<hhhH', self.receivedTestData)
                self.thePort = self.testPort
            except Exception as e:
                print(e)
        # communication established, keep channel open
        print("Mainboard communication established successfully on port: ", self.thePort)

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

'''
### find all available ports, check which works for mainboard comms, return that port 
def port_check():
    speed1 = speed2 = speed3 = thrower_speed = disable_failsafe = 0
    data = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            testPort = "/dev/"+port.name # serial.Serial needs specific notation to open a serial com port
            print("Testing on port: ",testPort)
            testSerialPort = serial.Serial(testPort, baudrate = 115200, timeout = 2) # test serial port with provided port from list of ports available
            testSerialPort.write(data)
            receivedData = testSerialPort.read(dataSize)
            data = struct.unpack('<hhhH', receivedData)
            thePort = testPort
        except Exception as e:
            print(e)
        finally:
            testSerialPort.close() # this was test comms to check which port worked so this channel needs to be closed after testing...
    print("Mainboard communication established successfully on port: ", thePort)
    return thePort


dataSize = struct.calcsize('<hhhH')

mainboardPort = port_check()
'''