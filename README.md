PIC_MULTI_10_IO
===============

RS-485 module with 10  I/O multi-function

Multi-Purpose RS-485 with Modbus Protocol using a small cpu PIC16F1827


  The PIC  C code 
  
    - main.c             Main Source code.
    - IOCycle.c          Sensor cycle management.
    - DHT22.c            DHT11, DHT22 and AM2303 Temperature Sensor code.
    - DS18B20.c          Digital DS18B20 sensor code.
    - RCServo.c          R/C servo  PWM code.
    - CAPSENSE.c         Capacitive sensor code.
    - CRC16.c            Modbus Cyclic redundancy check code.

  The PIC Header
  
    - IOCycle.h
    - DHT22.h
    - DS18B20.h
    - RCServo.h
    - IOCONFIG.h         IO , Cpu Frequency and timing information header.
 

  The binary file

    - Pic_Multi_10.hex   Compile code in hex format.


  Add-on file for Raspberry Pi

    - PicModule.py        Python class interface.
    - configPIC.c         Application to configure each remote module.
    - test.c              Example code to interface the remote with libmodbus in C.


  The Schematic

    - MultiIO10.png  Electronic schematic.

  The GPL License

    - gpl.txt

   
