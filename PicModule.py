#!/usr/bin/python3.2
import minimalmodbus
import time


class PicMbus:


  #CONFIGURATION IO
  IOCONFIG_NONE= 255
  IOCONFIG_ANALOGVDD= 0
  IOCONFIG_ANALOG1V=  1
  IOCONFIG_ANALOG2V=  2
  IOCONFIG_ANALOG4V=  3
  IOCONFIG_INPUT=     4
  IOCONFIG_INPUT_PULLUP= 5
  IOCONFIG_OUTPUT=    6
  IOCONFIG_PWM=       7
  IOCONFIG_CAP_SENSE_OFF= 8
  IOCONFIG_CAP_SENSE_LOW=9
  IOCONFIG_CAP_SENSE_MEDIUM=10
  IOCONFIG_CAP_SENSE_HIGH=11
  IOCONFIG_DHT11=     16
  IOCONFIG_DHT22=     17
  IOCONFIG_DS18B20=   32
  IOCONFIG_SERVO=     64
  IOCONFIG_COUNTER=   128
  IOConfig=[IOCONFIG_NONE,IOCONFIG_NONE,IOCONFIG_NONE,IOCONFIG_NONE,IOCONFIG_NONE,
            IOCONFIG_NONE,IOCONFIG_NONE,IOCONFIG_NONE,IOCONFIG_NONE,IOCONFIG_NONE]

  def __init__(self,SlaveAddress,Baud=57600,Device='/dev/ttyUSB0'):
    self.SlaveAddress= SlaveAddress
    self.module = minimalmodbus.Instrument(Device,SlaveAddress)
    self.module.serial.baudrate=Baud
    self.module.serial.timeout=0.05
    self.module.serial.flushInput();
    #ok lire Identification
#    Id = self.readId()
 
#    Count=0
#    if  Id == 0x653A:
    Count=10
#    else:
#     Count=2

    #ok lire Configuration
#   for loop in range(Count):
#     self.IOConfig[loop]= self.module.read_register(0x100+loop,0,3)

  def setAddress(self, SlaveAddress):
    self.SlaveAddress = SlaveAddress
    self.module.address=SlaveAddress
    #we change module . we don't know the io config so let's put it to none
    for i in self.IOConfig:
       i=self.IOCONFIG_NONE

  def config(self,Pin,value):
    #enable configuration change
    self.module.write_register(0x1ff,0x5678,0,6)
    self.module.write_register(0x100+Pin,value,0,6)
    self.IOConfig[Pin]= value

  def readConfig(self,Pin):
    ioconfig=self.module.read_register(0x100+Pin,0,3)
    self.IOConfig[Pin]=ioconfig
    return ioconfig    

  def readVRef2V(self):
    return self.module.read_register(0x1000,0,4)

  def readDiode(self):
    return self.module.read_register(0x1001,0,4)

  def readSensor(self,Pin):
    #first let's checkif we know the config information
    ioconfig= self.IOConfig[Pin]

    if ioconfig == self.IOCONFIG_NONE:
      ioconfig= self.readConfig(Pin)  

    if (ioconfig == self.IOCONFIG_PWM) or (ioconfig == self.IOCONFIG_SERVO):
     return self.module.read_register(Pin);
    elif (ioconfig & self.IOCONFIG_CAP_SENSE_OFF)>0:
      return self.module.read_long(Pin*16,4,False)

    elif (ioconfig & (self.IOCONFIG_DHT11 | self.IOCONFIG_DS18B20 | self.IOCONFIG_COUNTER))>0:
      return self.module.read_registers(Pin*16,3,4)
    else:
      return self.module.read_registers(Pin*16,1,4)[0]

  def resetCounter(self,Pin):
     return self.module.write_register(Pin,0,0,6)

  def ResetModule(self):
     return self.module.write_register(0xAA55,0x1234,0,6)

  def RCServo(self,Pin,value):
     return self.module.write_register(Pin,value,0,6)


  def readVersion(self):
     version = self.module.read_register(250,0,3)
     print("{}.{}".format(version>>8, version & 0xff))


  def readId(self):
     return self.module.read_register(251,0,3)

  def readIO(self,Pin):
     return self.module.read_bit(Pin)

  def writeIO(self,Pin,Value):
     self.module.write_bit(Pin,Value)

  def writeAllIO(self,Value):
     self.module.write_register(0x1002,Value,0,6)

  def readAllIO(self):
     return self.module.read_register(0x1002,0,4)
  

  def readDHT(self,Pin):
     #check config
     if self.IOConfig[Pin] == self.IOCONFIG_NONE:
         self.readConfig(Pin)
     if (self.IOConfig[Pin] &  self.IOCONFIG_DHT11) == 0:
        #return IO  config not set for dht
        return None
     while(True):
        value = self.readSensor(Pin)
        if value[0] == 0xffff:
           #No sensor Found
           return None
        if value[0] == 1:
           if self.IOConfig[Pin] == self.IOCONFIG_DHT11 : 
              Factor = 1.0
           else:
              if(value[2] & 0x8000)!=0:
                Factor = (-0.1)
              else:
                Factor  = 0.1
           temperature = (value[2] & 0x7fff) * Factor
           humidity    = value[1] * 0.1 
           return [humidity , temperature]


  def readDS18B20(self,Pin):
     #check config
     if self.IOConfig[Pin] == self.IOCONFIG_NONE:
         self.readConfig(Pin)
     if (self.IOConfig[Pin] & self.IOCONFIG_DS18B20) ==0:
        #return IO  config not set for dht
        return None, 0xffff
     value = self.readSensor(Pin)
     Factor = 0.0625
     Temp = value[1]
     if(Temp & 0x8000) !=0:
        Temp-=65536
     return Temp*Factor, value[0]

