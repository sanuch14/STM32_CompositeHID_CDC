# STM32_CompositeHID_CDC USB device

This software is build using CubeMX and Keil for STM32F042K6T6.

Composite device tested on WINDOWS 7 x32/x64.
To start device STM32 Virtual COM Port Driver is required.
Because of PID is changed, You have to put "compositecdc.inf" file in to the 
directory, where standart STM32 COM port driver is installed.

HID part of device work with standart windows driver.