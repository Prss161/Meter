import stm32l476x_UART as stm # Importing modules 
import time

controller = stm.stm32l476x('COM3', 115200) # Iinitiazlizing controller

if __name__ == '__main__':
    controller.GetProcessData() # Geting all coded data such as temperature, pressure, ADC, height.
    time.sleep(5)
    controller.ChangeDisplay(input('Choose between logo, temp, pressure, height, adc: ')) # Changing displayed menu
    time.sleep(5)
    controller.ChangeProcessUnits(input('Choose between SetTempC, SetTempF: ')) # Selecting units between Celcius and Farenhait