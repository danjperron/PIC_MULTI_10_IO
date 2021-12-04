/**************************************
PIC Multi-Purpose Modbus

configuration application

This program is used to set the Modbus Address of the PIC Multi-purpose module

it will also set the IO pin mode configuration

to compile:

   gcc -I /usr/include/modbus configPIC.c -o configPIC  -l modbus

libmodbus is needed

 please check url: http://libmodbus.org

 You could  download libmodbus on the Raspberry Pi using

    sudo apt-get install libmodbus5 libmodbus-dev

PIC Multi-Purpose  configPIC  program
Daniel Perron (c)  April, 2014
V1.0
***************************************/

////////////////////////////////////  GPL LICENSE ///////////////////////////////////


/*
 This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <modbus.h>
#include <ctype.h>

#define DEFAULT_DEVICE "/dev/ttyUSB0"

typedef struct
{
unsigned short     mode;
unsigned char * asciiMode;
unsigned char SpecialFunctions;
}IOConfigStruct;

char device[256];
int Baud = 9600;

unsigned char scanOnly=0;
unsigned char CurrentSlaveAddress=255;
unsigned char CurrentSlaveId=0;
unsigned char CurrentSlaveIOCount=0;
long  timeout=0;

int IsModuleFound(modbus_t * mb, unsigned char  SlaveAddress);
unsigned char selectModule(modbus_t * mb);


#define IOCONFIG_ANALOGVDD 0
#define IOCONFIG_ANALOG1V  1
#define IOCONFIG_ANALOG2V  2
#define IOCONFIG_ANALOG4V  3
#define IOCONFIG_INPUT     4
#define IOCONFIG_INPUT_PULLUP 5
#define IOCONFIG_OUTPUT    6
#define IOCONFIG_PWM       7
#define IOCONFIG_CAP_SENSE_OFF    8
#define IOCONFIG_CAP_SENSE_LOW 	  9
#define IOCONFIG_CAP_SENSE_MEDIUM 10
#define IOCONFIG_CAP_SENSE_HIGH   11


#define IOCONFIG_DHT11     16
#define IOCONFIG_DHT22     17
#define IOCONFIG_DS18B20   32
#define IOCONFIG_SERVO     64
#define IOCONFIG_COUNTER   128
#define IOCONFIG_COUNTER_PULLUP   129


IOConfigStruct  configInfo[]={
  {  IOCONFIG_ANALOGVDD, "ANALOGVDD",0},
  {  IOCONFIG_ANALOG1V, "ANALOG1V",0},
  {  IOCONFIG_ANALOG2V, "ANALOG2V",0},
  {  IOCONFIG_ANALOG4V, "ANALOG4V",0},
  {  IOCONFIG_INPUT, "INPUT",0},
  {  IOCONFIG_INPUT_PULLUP, "INPUT PULLUP",1},
  {  IOCONFIG_OUTPUT, "OUTPUT",0},
  {  IOCONFIG_PWM,"PWM",2},
  {  IOCONFIG_CAP_SENSE_OFF, "CAP SENSE OFF",1},
  {  IOCONFIG_CAP_SENSE_LOW, "CAP SENSE LOW",1},
  {  IOCONFIG_CAP_SENSE_MEDIUM, "CAP SENSE MEDIUM",1},
  {  IOCONFIG_CAP_SENSE_HIGH, "CAP SENSE HIGH",1},
  {  IOCONFIG_DHT11, "DHT11",1},
  {  IOCONFIG_DHT22, "DHT22",1},
  {  IOCONFIG_DS18B20, "DS18B20",0},
  {  IOCONFIG_SERVO, "R/C SERVO",0},
  {  IOCONFIG_COUNTER, "COUNTER",1},
  {  IOCONFIG_COUNTER_PULLUP, "COUNTER PULLUP",1},
  {65535, "",0}};

unsigned char  IsPinHaveSpecialFunctions[]={1,1,1,1,1,0,0,0,0,0};
unsigned char  IsPinHavePWM[]=             {1,0,0,1,0,0,0,0,1,1};

uint16_t  MB_Version;


int moduleID =0;

int GetConfigIndex(int mode)
{
   int loop;
   for(loop=0;;loop++)
     {
       if(configInfo[loop].mode== 65535) return -1;
       if(configInfo[loop].mode== mode) return loop;
     }
 return -1;
}



int IsConfigValidOnPin(int mode, int Pin)
{
    int  configIndex;

    if(moduleID == 0) return 0;

    if(Pin<0) return 0;

    if(mode == IOCONFIG_COUNTER_PULLUP)
     {
      if(moduleID != 0x653A)
          if(Pin>1) return 0;
     }

    if(Pin>9) return 0;

    configIndex = GetConfigIndex(mode);

    if(configIndex <0) return 0;

    if(mode == IOCONFIG_PWM)
        return IsPinHavePWM[Pin];


    if(configInfo[configIndex].SpecialFunctions)
      if(IsPinHaveSpecialFunctions[Pin]==0)
         {
            return 0;
         }

    return 1;
}



char * configModeInText(int IOConfigMode)
{
   int loop;

   loop = GetConfigIndex(IOConfigMode);

//    printf("%03d ",IOConfigMode);

   if(loop>=0)
      return configInfo[loop].asciiMode;
   return "???";
}


void decodeArg(int argc, char * argv[])
{
  int loop;

  for(loop=1;loop<argc;loop++)
   {
    if(strcasecmp(argv[loop],"-scan")==0)
      scanOnly=1;
    if(strcmp(argv[loop],"-d")==0)
      {
       loop++;
       strcpy(device,argv[loop]);
      }
    if(strcmp(argv[loop],"-b")==0)
      {
        loop++;
        Baud=atoi(argv[loop]);
        if(Baud >0)
        if(timeout == 0)
        {
          timeout = (1000000 / Baud)*576;
        }

      }
    if(strcmp(argv[loop],"-t")==0)
       {
         loop++;
         timeout = atol(argv[loop]);
       }
    if(strcmp(argv[loop],"--help")==0)
      {
        printf("usage:\nconfigPIC  [-scan][-d DeviceName][-b BaudRate] [-t response_Delay_us]\n");
        exit(0);
      }
  }
}


void changeAddress(modbus_t * mb)
{

  int  new_Address;

  printf("\n=======Change Address\n");





  if(CurrentSlaveAddress == 255)
     selectModule(mb);
  else
     if(!IsModuleFound(mb,CurrentSlaveAddress))
       selectModule(mb);

  if(CurrentSlaveAddress==255) return;


   printf("\nEnter new Slave Address for this module (1..127) ?");

   if(scanf("%d",&new_Address)!=1)
      printf("Invalid value!  configuration not change.\n");
   else
     {
       if(new_Address>0)
        if(new_Address<128)
        {
          if(IsModuleFound(mb,new_Address))
          {
      	   modbus_set_slave(mb,CurrentSlaveAddress);
           printf("Address already used!\nConfiguration not changes.\n");
          }
          else
          {
   	   modbus_set_slave(mb,CurrentSlaveAddress);
           // unlock confiration
           modbus_write_register(mb,0x1ff,0x5678);
           modbus_write_register(mb,160,new_Address);
           printf("Module is now on Address %d\n",new_Address);
           CurrentSlaveAddress=new_Address;
          }
        }
      else
       printf("Invalid value!  configuration not change.\n");
     }
   usleep(1000000); // delay needed to store data into eerom
   IsModuleFound(mb,CurrentSlaveAddress);

}


unsigned short  getConfigMode(modbus_t * mb, int Pin)
{
     uint16_t MB_Register;
     if( modbus_read_registers(mb,0x100+Pin,1,&MB_Register) < 0)
        return 65535;
     return MB_Register;
}


int IsModuleFound(modbus_t * mb, unsigned char  SlaveAddress)
{
   int rcode;
   uint16_t  MB_Register;

   if(SlaveAddress >127) return 0;

   modbus_set_slave(mb,SlaveAddress);
   if (modbus_read_registers(mb,251,1,&MB_Register) <0)
       return 0;
   if (modbus_read_registers(mb,250,1,&MB_Version) <0)
       MB_Version=0;

   if(MB_Register == 0x6532)
     return 0x6532;
   if(MB_Register == 0x653A)
     return 0x653A;
   return 0;
}


unsigned char selectModule(modbus_t * mb)
{
   char buffer[256];
   int SlaveAddress;
   int rcode;
   uint16_t MB_Register;
   CurrentSlaveAddress=255;
   CurrentSlaveIOCount=0;
   CurrentSlaveId=0;
   printf("Select Module\nEnter Slave Address ?");

   if(scanf("%d",&SlaveAddress)==1)
      if(SlaveAddress>0)
       if(SlaveAddress<128)
         {
           CurrentSlaveId=IsModuleFound(mb,SlaveAddress);
           CurrentSlaveIOCount = CurrentSlaveId & 0xf;
           CurrentSlaveAddress=SlaveAddress;
          }
  printf("\n");fflush(stdout);
  return SlaveAddress;
}



void changeConfigMode(modbus_t * mb,int Pin)
{
  int loop;
  unsigned short  mode;
  unsigned char  module;
  int  new_mode;
  uint16_t MB_mode;

  printf("\n=======  Change  IO%d mode\n",Pin);




  if(CurrentSlaveAddress == 255)
     selectModule(mb);
  else
     if(!IsModuleFound(mb,CurrentSlaveAddress))
       selectModule(mb);
  if(CurrentSlaveAddress==255) return;

  mode=getConfigMode(mb,Pin);


  for(loop=0;;loop++)
    {

      if(configInfo[loop].mode==65535) break;

      if(loop%3==0)
        printf("\n");

      if(IsConfigValidOnPin(configInfo[loop].mode,Pin))
      printf("%3d) %-20s",configInfo[loop].mode, configInfo[loop].asciiMode);
      else
      printf("%3s  %-20s"," "," ");
      fflush(stdout);
    }


  printf("\n\nSlave address %d  current IO%d mode is %d : %-20s\n",CurrentSlaveAddress,Pin,mode,configModeInText(mode));

   printf("\nEnter new configuration ?");
   if(scanf("%d",&new_mode)!=1)
      printf("Invalid value!  configuration not change.\n");
   else
     {
      if(IsConfigValidOnPin(new_mode,Pin))
        {
          MB_mode= new_mode;
          // Enable configuration change
          modbus_write_register(mb,0x1ff,0x5678);
          modbus_write_register(mb,0x100+Pin,MB_mode);
         printf("Module %d  IO%d set to %d: %s\n",CurrentSlaveAddress,Pin,MB_mode,configModeInText(MB_mode));
        }
      else
       printf("Invalid value!  configuration not change.\n");
     }
   usleep(1000000); // delay needed to store data into eerom
}



void ReadDS18B20(modbus_t * mb,int _io)
{
    float Factor=0.0625,Temperature;
    int mask;
    short Temp;
    uint16_t  MB_Register[3];


    if(modbus_read_input_registers(mb,_io*16,3,MB_Register)>=0)
       {
         if(MB_Register[0]==0)
          {
            printf("Buzy");
          }
         else if (MB_Register[0]==1)
         {
           mask = MB_Register[2] & 0x60;
           Temperature = Factor * ((short)MB_Register[1]);
           printf("Temp: %5.1f Celsius",Temperature);
         }
         else if (MB_Register[0]==2)
         {
           printf("CRC Error");
         }
         else printf("Error");
       }
     else
      printf("Unable to read  Sensor");
     printf("\n");
}


void ServoData(modbus_t * mb, unsigned char _io)
{
  uint16_t MB_Register;
  int loop;
  if(modbus_read_registers(mb,_io,1,&MB_Register)<0)
     printf("Unable to read data\n");
  else
  {
      printf("[%d (0x%04X)] ",MB_Register,MB_Register);
    printf("\n");
  }
}


void RawData(modbus_t * mb, unsigned char _io, unsigned char Size)
{
  uint16_t MB_Register[3];
  int loop;
  if(modbus_read_input_registers(mb,_io*16,Size,MB_Register)<0)
     printf("Unable to read data\n");
  else
  {
    for(loop=0;loop<Size;loop++)
      printf("[%d (0x%04X)] ",MB_Register[loop],MB_Register[loop]);
    printf("\n");
  }
}

void  ReadDHT22(modbus_t * mb,int _io)
{
    float Factor,Temperature,Humidity;
    static char status[256];
  uint16_t  MB_Register[3];

//    printf("%-20s","DHT22");

    if(modbus_read_input_registers(mb,_io*16,3,MB_Register)>=0)
       {
         if(MB_Register[0]==0)
          {
            printf("Buzy");
          }
         else if (MB_Register[0]==1)
         {
           if(MB_Register[2] & 0x8000)
             Factor = (-0.1);
        else
             Factor = (0.1);
        Temperature = (MB_Register[2] & 0x7fff) * Factor;
        Humidity = (MB_Register[1] * 0.1);
        printf("Temp: %5.1f Celsius   Humidity: %5.1f%%",
          Temperature,Humidity);
         }
         else printf("Error");
       }
     else
      printf("Unable to read DHT22 Sensor");
    printf("\n");
}


void  ReadDHT11(modbus_t * mb,int _io)
{
    float Temperature,Humidity;
    static char status[256];
  uint16_t  MB_Register[3];


    if(modbus_read_input_registers(mb,_io*16,3,MB_Register)>=0)
       {
         if(MB_Register[0]==0)
          {
            printf("Buzy");
          }
         else if (MB_Register[0]==1)
         {
        Temperature = (MB_Register[2] & 0xff);
        Humidity = (MB_Register[1] &0xff);
        printf("Temp: %5.0f Celsius   Humidity: %5.0f%%",
          Temperature,Humidity);
         }
         else printf("Error");
       }
     else
      printf("Unable to read DHT11 Sensor");
    printf("\n");
}


void displaySensorData(modbus_t * mb, int _io)
{
uint16_t IOConfig;

// read io configuration
     if(modbus_read_registers(mb,0x100+_io,1,&IOConfig)<0)
       {
         printf(" Unable to read configuration\n");
         return;
       }

     switch(IOConfig)
      {
       case IOCONFIG_ANALOGVDD: RawData(mb,_io,1);break;
       case IOCONFIG_ANALOG1V:  RawData(mb,_io,1);break;
       case IOCONFIG_ANALOG2V:  RawData(mb,_io,1);break;
       case IOCONFIG_ANALOG4V:  RawData(mb,_io,1);break;

       case IOCONFIG_INPUT:
       case IOCONFIG_INPUT_PULLUP:
       case IOCONFIG_OUTPUT:     RawData(mb,_io,1);break;

       case IOCONFIG_CAP_SENSE_OFF:
       case IOCONFIG_CAP_SENSE_LOW:
       case IOCONFIG_CAP_SENSE_MEDIUM:
       case IOCONFIG_CAP_SENSE_HIGH:  RawData(mb,_io,2);break;


       case IOCONFIG_DHT11:       ReadDHT11(mb,_io);break;
       case IOCONFIG_DHT22:       ReadDHT22(mb,_io);break;
       case IOCONFIG_DS18B20:     ReadDS18B20(mb,_io);break;
       case IOCONFIG_SERVO:
       case IOCONFIG_PWM:         ServoData(mb,_io);break;
       case IOCONFIG_COUNTER_PULLUP:
       case IOCONFIG_COUNTER:    RawData(mb,_io,3);break;
       default:   printf("\n");
      }
}






void DisplayIOConfig(modbus_t * mb)
{
  int loop,io;
  int rcode;
  uint16_t  MB_Register;
  int configMode;
  double VDD;

  if(CurrentSlaveAddress==255)
      return;


    moduleID = IsModuleFound(mb,CurrentSlaveAddress);

   if(moduleID)
   {

    printf("=============\n%d : ",CurrentSlaveAddress);

    printf("Type %04X  Multi Purpose %dIO", moduleID, moduleID & 0xf);
    printf(" , Version:%d.%02d\n",MB_Version >>8, MB_Version & 0xff);


    for(io=0;io<(moduleID & 0xf);io++)
    {
      printf("%5sIO%d: ","",io);
      configMode= getConfigMode(mb,io);
      printf("%-20s",configModeInText(configMode));
      // check if config is valid
      if(IsConfigValidOnPin(configMode,io))
        displaySensorData(mb,io);
      else
        printf("*** INVALID CONFIGURATION ***\n");
    }

    if(moduleID == 0x653A)
    {
      uint16_t MB_Register[3];
      printf("\n");
      // get VDD
      if(modbus_read_input_registers(mb,0x1000,1,MB_Register)>=0)
	{
          if(MB_Register[0]>0)
          {
           VDD = 2.048 * 1023 / MB_Register[0];
           printf("%5sVDD: %5.2fV","",VDD);
           // get DIODE A/D
           if(modbus_read_input_registers(mb,0x1001,1,MB_Register)>=0)
	    {
                 printf("     Diode A/D: %d\n",MB_Register[0]);
            }
          }
       }
    }
    printf("\n");
   }
}


void scanBus(modbus_t * mb)
{
  int loop,io;
  int rcode;
  int Id;
  uint16_t  MB_Register;


  printf("\n==== Scanning MODBUS\n");fflush(stdout);


  for(loop=1;loop<128;loop++)
  {

    Id = IsModuleFound(mb,loop);

   if(Id)
   {

    printf("%d : ",loop);

    printf("Type %04X  Multi Purpose %dIO\n", Id, Id & 0xf);

/*
    Id &= 0xf;

    for(io=0;io<Id;io++)
    {
      printf("IO%d: ",io);
      printf("%-20s",configModeInText(getConfigMode(mb,io)));
    }
    printf("\n");
*/   }
 }



}


unsigned char ReadOneKey(void)
{
  char c;
  struct termios term,term_orig;

  // put keyboard in raw mode

  tcgetattr(0,&term_orig);
  memcpy(&term,&term_orig,sizeof(struct termios));
  term.c_lflag &= ~(ICANON | ECHO);
  term.c_cc[VTIME] =0;
  term.c_cc[VMIN]=1;
  tcsetattr(0,TCSANOW, &term);

  while(read(0, &c,1)!=1);

  tcsetattr(0,TCSANOW,&term_orig);
  fflush(stdin);
   return c;
}

void ResetModule(modbus_t  * mb)
{
   char rcode;
   char loop;
   printf("Reset Module (Y/N)?");fflush(stdout);
   rcode = toupper(ReadOneKey());
   if(rcode!='Y') return;
   modbus_write_register(mb,0xAA55,0x1234);
   printf("\n***** Resetting module ****\nPlease wait");fflush(stdout);
   for(loop=0;loop<10;loop++)
     {
      usleep(500000);
      printf(".");fflush(stdout);
     }
  printf("\n");
  modbus_flush(mb);
}

void SetOutputSignal(modbus_t * mb)
{
  char rcode;
  int SelectedIO;
  uint16_t MB_Register;
  int value;
  printf("**** works for Output , PWM and R/C Servos only\n");
  printf("Select IO (0..9)?");fflush(stdout);

  rcode = toupper(ReadOneKey());

  if(rcode  < '0') return;
  if(rcode > '9') return;

  SelectedIO = rcode - '0';

  printf("\nEnter Data signal ?");fflush(stdout);
  if(scanf("%d",&value)==1)
  {
   MB_Register=value;
   modbus_write_register(mb,SelectedIO,MB_Register);
  }
  printf("\n");
}


void Menu(modbus_t * mb)
{
char  rcode;
int SelectedIO;

 printf("\nPIC multi-purpose I/O  MODBUS configuration\n");
 printf("Version 1.0 (c) Daniel Perron, April 2014\n");
 printf("device:%s Baud Rate:%d timeout:%uus\n",device,Baud,timeout);
 while(1)
  {
    // print menu
    printf("\n\nM) MODBUS scan \n");
    printf("F) FLush  buffer\n");
    printf("A) Select Slave Module\n");
    if(CurrentSlaveAddress!=255)
     {
      printf("I) DispLay IO configuration\n");
      printf("S) Set Output signal\n");
      printf("R) Reset Module\n");
     }
    printf("C) Change Slave Address\n");
    printf("0..9) Set IO mode\n");
    printf("Q) Quit\n");


    printf("\nSelected module :");

    if(CurrentSlaveAddress == 255)
     printf("None\n");
    else
     printf("%d\n",CurrentSlaveAddress);

    fflush(stdout);

    rcode= toupper(ReadOneKey());
    fflush(stdin);
    switch(rcode)
     {
     case 'M':  scanBus(mb);break;
     case 'F':  modbus_flush(mb);
                printf("Flush Received data....");
                fflush(stdout);
                usleep(500000);
                break;
     case 'I':  if(CurrentSlaveAddress!=255)
                  DisplayIOConfig(mb);
                break;
     case 'A':  selectModule(mb);break;
     case 'C':  changeAddress(mb);break;
     case 'S':  if(CurrentSlaveAddress != 255)
		SetOutputSignal(mb);
                break;
     case 'R': if(CurrentSlaveAddress != 255)
                ResetModule(mb);
                break;
     case '0':
     case '1':
     case '2':
     case '3':
     case '4':
     case '5':
     case '6':
     case '7':
     case '8':
     case '9':
               SelectedIO= rcode -'0';
               if(CurrentSlaveIOCount < SelectedIO)
                  return;
               changeConfigMode(mb,SelectedIO);
                break;
     case 'Q':  return;
    }
  }

}



int main(int argc, char * argv[])
{

   modbus_t *mb;

  strcpy(device,DEFAULT_DEVICE);

  decodeArg(argc,argv);
  timeout = ((timeout + 500)  / 1000) * 1000;
  if(timeout ==0)
     timeout=10000;
  if(timeout > 999900)
     timeout=999900;

   mb = modbus_new_rtu(device,Baud,'N',8,1);
   modbus_connect(mb);

//set 1/100th of second response time
//   struct timeval response;

// new version used directly uint32_t variable
//   response.tv_sec=0;
//   response.tv_usec=timeout;
// modbus_set_response_timeout(mb, &response);
modbus_set_response_timeout(mb,0, (uint32_t) timeout);


if(scanOnly==1)
  scanBus(mb);
else
   Menu(mb);

 modbus_close(mb);
 modbus_free(mb);
 return 0;
}
