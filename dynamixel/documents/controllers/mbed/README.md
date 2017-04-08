mbed Controller Information (for intfc. with Dynamixel AX-12A)
===============================

This directory has information regarding the use of the mbed controller (mbed NXP LPC1768) to command and communicate with Robotis Dynamixel motors (eg. AX-12A)

 Directory | Contents
 --------- | --------
mbed_ax12_test_full | Example mbed program (source and libraries) to command goal position of an AX-12A motor (import mbed_ax12_test_full.zip into mbed IDE)

Files and libraries were used & modified from the following sources:

 Library | Description | URI
 ------- | ----------- | ---
AX-12 library | Provides class/functions to access AX-12A control table (eg. id, baud, goal_position, etc.) | http://mbed.org/cookbook/Dynamixel-AX12-Servo
SerialHalfDuplex library | Deprecated serial library needed by AX12.h/.c to allow mbed to communicate with AX-12A | https://mbed.org/users/mbed_official/code/mbed/file/e2ed12d17f06/SerialHalfDuplex.h
GPIO library | Provided availalbe gpio functions with which to update SerialHalfDuplex .h/.c | http://mbed.org/users/mbed_official/code/mbed/file/a9913a65894f/gpio_api.h; https://github.com/mbedmicro/mbed/blob/master/libraries/mbed/targets/hal/TARGET_STM/TARGET_STM32F4XX/gpio_api.c

NOTE: SerialHalfDuplex.h/.c were modified to replace deprecated gpio function calls
