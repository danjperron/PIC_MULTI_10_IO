#!/usr/bin/python3.2
import sys
import webiopi
import threading

sys.path.append("/home/pi")
import PicModule
import time
import datetime
import os
import sys
import shlex
import subprocess

CurrentMin=0
PreviousMin=255

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


#ajout de adresse 0 dans le pic.
#ce qui veut dire tout module.
@webiopi.macro
def allOFF(Module):
  modules.setAddress(0)
  modules.writeAllIO(0)


class RecordTemperature(threading.Thread):
  def __init__(self,Temperature):
      threading.Thread.__init__(self)
      self.Temperature = Temperature
      self.fichierRrdtool = "/home/pi/data/temperatures.rrd"

  def LireCpu(self):
      fichier = open("/sys/class/thermal/thermal_zone0/temp","r")
      texte =  fichier.readline()
      fichier.close()
      return  (float(texte)/1000.0)


  def rrdExport(self,debut , step , sortieXML):
      texte = "rrdtool xport -s {0} -e now --step {1} ".format(debut, step)
      texte += "DEF:a={0}:th_cpu:AVERAGE ".format(self.fichierRrdtool)
      texte += "DEF:b={0}:th_t1:AVERAGE ".format(self.fichierRrdtool)
      texte += "XPORT:a:""cpu"" "
      texte += "XPORT:b:""Capteur1"" "
      sortie = open("/usr/share/webiopi/htdocs/temperature/{0}".format(sortieXML),"w")
      args = shlex.split(texte)
      subprocess.Popen(args, stdout=sortie)
      sortie.close()  



  def ValideValeur(self,valeur):
    if valeur == None:
      return ":U"
    else:
      return ":{0:.1f}".format(valeur)



  def run(self):
      # enregistrer La temperature du cpu
      cpuTemperature=self.LireCpu()
      
      #updatons rddtool
      
      texte = "N" + self.ValideValeur(cpuTemperature) + self.ValideValeur(self.Temperature)
      #inserons dans le fichier data
      subprocess.Popen(["/usr/bin/rrdtool","update",self.fichierRrdtool,texte])

      #maintenant Creons les donnees pour 3 heures data
      self.rrdExport("now-3h",300,"temperature3h.xml")

      #24 heures
      self.rrdExport("now-24h",900,"temperature24h.xml")

      #48 heures
      self.rrdExport("now-48h",1800,"temperature48h.xml")

      #1 semaine
      self.rrdExport("now-7d",3600,"temperature1w.xml")
 
      #1 mois
      self.rrdExport("now-1month",14400,"temperature1m.xml")

      #3 mois
      self.rrdExport("now-3month",28800,"temperature3m.xml")
   
      #1 an
      self.rrdExport("now-1y",43200,"temperature1y.xml")


def loop():
  global CurrentMin
  global PreviousMin
  Temperature=-999
  now= datetime.datetime.now()
  CurrentMin= now.minute
  if not(CurrentMin == PreviousMin):
    PreviousMin = CurrentMin
    modules.setAddress(11)
    Temperature= modules.readDS18B20(8)
    if not(Temperature == None):
      try:
        fichier = open("/usr/share/webiopi/htdocs/temperature/temperature.txt","w")
        fichier.write("{0}\t{1:.1f}\n".format(now.strftime("%H:%M:%S"),Temperature))
        fichier.close()
      except:
        pass
    recordThread = RecordTemperature(Temperature)
    recordThread.start()
  webiopi.sleep(1)           
