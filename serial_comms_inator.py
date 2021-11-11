import struct
import serial
import serial.tools.list_ports

### find all available ports, check which works for mainboard comms, return that port 
def port_check():
    speed1 = speed2 = speed3 = thrower_speed = disable_failsafe = 0
    data = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            test_port = "/dev/"+port.name # serial.Serial needs specific notation to open a serial com port
            print("Testing on port: ",test_port)
            test_ser = serial.Serial(test_port, baudrate = 115200, timeout = 2) # test serial port with provided port from list of ports available
            test_ser.write(data)
            received_data = test_ser.read(data_size)
            data = struct.unpack('<hhhH', received_data)
            the_port = test_port
        except Exception as e:
            print(e)
        finally:

            test_ser.close() # this was test comms to check which port worked so this channel needs to be closed after testing...
    print("Mainboard communication established successfully on port: ", the_port)
    return the_port


data_size = struct.calcsize('<hhhH')


mainboardPort = port_check()


# class instance that handles mainboard communication neatly
class Comm_Mainboard():
    def __init__(self):
        self.ser = serial.Serial(mainboardPort, baudrate = 115200, timeout = 2)
    def SendCmd2Mbd(self, speed1, speed2, speed3, thrower_speed, disable_failsafe):
        try: # despite already having checked if the port is accessible, use try in case of HW failures
            data = struct.pack('<hhhHBH', speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)
            self.ser.write(data)
            received_data = self.ser.read(data_size)
            data = struct.unpack('<hhhH', received_data)
        except Exception as e:
            raise e() # if HW failure happens, will likely raise this error at this point