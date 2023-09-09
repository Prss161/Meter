# ---------------IMPORTS---------------
from uart_base import UartCOM
from iUart import iUart
import time

# ---------------PROGRAM STARTS---------------
class stm32l476x_UART(UartCOM, iUart):
    
    def __init__(self, port: str , baudrate: int):
        '''
        Description:
        This class handles communication beetween UART and STM32 controller
        
        Args:
        str: port - COM port that controller is connected
        int: baudrate - baudrate on which UART is communicating with controller
        
        Return:
        None
        '''         
        self._objUART = UartCOM(port, baudrate)
        return

    def GetProcessData(self, delay = 1): 
        '''
        Description:
        This function prints process data inside python terminal sucha as temperature
        pressure, adc, height
        
        Args:
        int: delay - delay beetween functions
        '''
        self._objUART.send('GetUnits')
        time.sleep(delay)
        process_data = self._objUART.listen()
        return process_data

    def ChangeProcessUnits(self, msg: str):
        '''
        Description:
        This function changes units and save the changes on eeprom, for example temperature in C
        to F and etc.
        
        Args:
        str: msg - predefined message such as (SetTempC,SetTempF)
        '''        
        ProcessUnits_commands = {
            'SetTempC' : 'SetTempC',
            'SetTempF' : 'SetTempF'
        }
        if msg in ProcessUnits_commands:
            self._objUART.send(ProcessUnits_commands[msg])
        time.sleep(1)
        self.WriteToEeprom()
        print(f'changed units in: {msg}')
    
    def ChangeDisplay(self,msg: str):
        '''
        Description:
        This function changes displayed menu on lcd screen.
        
        Args:
        str: msg - predefined message such as (logo, temperature, pressure)
        
        ''' 
        display_commands = {
            'logo': 'logo',
            'temp': 'temp',
            'pressure': 'pressure',
            'height': 'height',
            'adc': 'adc'
        }
        if msg in display_commands:
            self._objUART.send(display_commands[msg])
        print(f'Changing menu display to {msg}')
    
    def WriteToEeprom(self):
        '''
        Description:
        This function writes data on eeprom
        ''' 
        self._objUART.send('WriteToEeprom')
