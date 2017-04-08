Dynamixel_IO
=====

Matlab class to command Robotis Dynamixel motors: AX-*, RX-*. Utilizes serial port to send commands and recieve motor status. Dynamixel_IO provides convenient member methods covering all entries defined in the motors' register tables (ie. all high level motor functions). It also provides access for construction, sending and receiving of low level packets to/from the Dynamixel motors.


 Requires:
 ---------
OpenCM 9.04 (optionally with OpenCM 485 EXP) loaded with tosser code
FTDI USB-Serial adapter between PC and OpenCM 9.04
