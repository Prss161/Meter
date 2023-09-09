# Temperature, pressure, height meter with STM32l476x

## Introduction
This device allows for displaying basic information about it's height, atmospheric pressure and temperature with functionality to switch units. 
Python modules have been used to communicate with controller via UART.

## Getting Started
Python 3.11.4 64-bit is required to run this project.
Additional python requirements are available in requirements.txt.

## Architecture

Class structure of python packages:

![Architecture](./python_interface/Doc/Img/Architecture.png)

## Demo

To test your device and run visual demo for all available functions use:
```bash
$ python python_interface/example.py
 ```

## Photos

##### Circuit diagram:

![Electric]()

##### Device pictures:

![Device1](./python_interface/Doc/Img/Device1.jpg)
![Device2](./python_interface/Doc/Img/Device2.jpg)
![Device3](./python_interface/Doc/Img/Device3.jpg)

#### Display:

![Temperature_C](./python_interface/Doc/Img/Temperature_C.jpg)
![Temperature_F](./python_interface/Doc/Img/Temperature_F.jpg)
![Pressure_Pa](./python_interface/Doc/Img/Pressure_Pa.jpg)
![Pressure_hPa](./python_interface/Doc/Img/Pressure_hPa.jpg)
![Height](./python_interface/Doc/Img/Height.jpg)
![ADC](./python_interface/Doc/Img/ADC.jpg)




