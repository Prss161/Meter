# ---------------IMPORTS---------------

import serial
import time
from iUart import iUart

# ---------------PROGRAM STARTS---------------

class UartCOM(iUart):
    
    def __init__(self, port: str, baudrate: int):
        '''
        Description:
        This class sets basic parameters for communication
        
        Args:
        str: port - COM port that controller is connected
        int: baudrate - baudrate on which UART is communicating with controller
        
        Return:
        None
        ''' 
        self.port = port
        self.baudrate = baudrate
        self.timeout = 0.01
        self.stop_bits = 1
        self.parity = serial.PARITY_NONE
        self.uart = serial.Serial(self.port,self.baudrate,timeout=self.timeout)
        self.uart.parity = self.parity
        self.uart.stopbits = self.stop_bits
        pass
    
    def listen(self):
        '''
        Description:
        Function prints messages that comes from controller
        '''
        lst = []
        while True:
            serial_out = self.uart.readline().decode('utf-8').strip()
            if not serial_out:
                break
            lst.append(serial_out)
            print(serial_out)
        return lst

    def send(self, msg: str, delay = 0.1):
        '''
        Description:
        This function sends messages to controller  
        
        Args:
        str: msg - string of letters (message) that is send to controller
        int: delay -  delay that each byte of data is send to controller
        '''
        for i in msg:
            self.uart.write(i.encode('ascii'))
            time.sleep(delay)
        self.uart.write('\n'.encode('ascii'))
    
    def ClearBuffer(self):
        '''
        Description:
        Function for clearing data at buffer
        '''
        while self.uart.in_waiting:
            _ = self.uart.read(self.uart.in_waiting)
    
    def Open(self):
        '''
        Description:
        Function used for opening com port
        '''
        self.uart.open()
        
    def Close(self):
        '''
        Description:
        Function used for closing com port
        '''
        self.uart.close()

# ---------------PROGRAM ENDS---------------