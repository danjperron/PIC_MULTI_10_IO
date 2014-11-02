#!/usr/bin/python3.2
import sys
import webiopi
sys.path.append("/home/pi")
import PicModule
import time
#import datetime

modules  = PicModule.PicMbus(127,Device='/dev/ttyUSB0')

ModuleAddress = [11,12,13]

@webiopi.macro
def setPin(Module,Pin, Value):
  modules.setAddress(ModuleAddress[int(Module)])
  modules.writeIO(int(Pin),int(Value))
  return(readPin(Module,Pin))
  
@webiopi.macro
def readPin(Module,Pin):
  time.sleep(0.05)
  modules.setAddress(ModuleAddress[int(Module)])
  Value = modules.readIO(int(Pin))
  return str(Value)

@webiopi.macro
def readAllPins(Module):
  modules.setAddress(ModuleAddress[int(Module)])
  Value = modules.readAllIO()
  return str(Value)

@webiopi.macro
def setAllPins(Module,Value):
  modules.setAddress(ModuleAddress[int(Module)])
  modules.writeAllIO(int(Value))


@webiopi.macro
def togglePin(Module,Pin):
  # read io sensor and inverse it 
  # not 0 = true not 1= false 
  # int(true)=1 int(false)=0
  modules.setAddress(ModuleAddress[int(Module)])
  Value = int( not  modules.readIO(int(Pin)))
  modules.writeIO(int(Pin),Value)
  return(readPin(Module,Pin))


#@webiopi.macro
#def allOFF(Module):
#  modules.setAddress(ModuleAddress[int(Module)])
#  for i in range(8):
#    modules.writeIO(i,0)

@webiopi.macro
def allOFF(Module):
  modules.setAddress(0)
  modules.writeAllIO(0)
  
