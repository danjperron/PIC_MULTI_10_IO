/**************************************
PIC Multi-Purpose Modbus

Test program demonstration on how to use libmodbus
with the  Pic Multi-Purpose remote

to compile:

   gcc -I /usr/include/modbus test.c -o test  -l modbus

PIC Multi-Purpose  test program
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
#include <time.h>
#include <modbus.h>


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

void  ReadDHT22(modbus_t * mb,int _io)
{
    float Factor,Temperature,Humidity;
    static char status[256];
  uint16_t  MB_Register[3];

    printf("%-20s","DHT22");

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

void ReadDS18B20(modbus_t * mb,int _io)
{
    float Factor,Temperature;
    int mask;
    short Temp;
    uint16_t  MB_Register[3];

    printf("%-20s","DS18B20");

    if(modbus_read_input_registers(mb,_io*16,3,MB_Register)>=0)
       {
         if(MB_Register[0]==0)
          {
            printf("Buzy"); 
          }
         else if (MB_Register[0]==1)
         {
           mask = MB_Register[2] & 0x60;
           switch(mask)
           {
            case 0 :   Factor = 0.0625/8.0;break;
            case 0x20: Factor = 0.0625/4.0;break;
            case 0x40: Factor = 0.0625/2.0;break;
            default:   Factor = 0.0625;
           }
           Temperature = Factor * ((short)MB_Register[1]);
           printf("Temp: %5.1f Celsius",Temperature);
         }
         else printf("Error");
       }
     else
      printf("Unable to read  Sensor");
     printf("\n");
}

void ReadTMP35(modbus_t * mb,int _io)
{
  uint16_t Temp;

    printf("%-20s","ANALOG 1V TMP35");
    if(modbus_read_input_registers(mb,_io*16,1,&Temp)>=0)
      printf("Temp: %5.1f Celsius\n",0.1 * Temp);
    else
      printf("Unable to read  input register\n");


}


void RawData(modbus_t * mb, unsigned char _io, char * Label, unsigned char Size)
{
  uint16_t MB_Register[3];
  int loop;
    printf("%-20s",Label);
  if(modbus_read_input_registers(mb,_io*16,Size,MB_Register)<0)
     printf("Unable to read\n",Label);
  else
  {
    
    for(loop=0;loop<Size;loop++)
      printf("[%d (0x%04X)] ",MB_Register[loop],MB_Register[loop]);
    printf("\n");
  }
}

void RegisterData(modbus_t * mb, unsigned char _io, char * Label)
{
  uint16_t MB_Register;
  int loop;
    printf("%-20s",Label);
  if(modbus_read_registers(mb,_io,1,&MB_Register)<0)
     printf("Unable to read\n",Label);
  else
      printf("[%d (0x%04X)]\n",MB_Register,MB_Register);
}



void PrintModule(modbus_t * mb,unsigned char _module)
{
   uint16_t MB_Register[3];
//   uint16_t ModeIO[10];
   int _io;
   int IoCount;
   int Flag=1;
   printf("     Module[%3d]",_module);

   // Set Slave module
     modbus_set_slave(mb,_module);

   // Get ID
     IoCount=0;
     if(modbus_read_registers(mb,251,1,MB_Register))
       {
         printf(" ID:%04X ",MB_Register[0]);
         if(MB_Register[0]==0x653A)
          IoCount=10;
         if(MB_Register[0]==0x6531)
          IoCount=2;
       }

    if(IoCount == 0)
       {
         printf("Unable to get module Id\n");
         return;
       }


     if(modbus_read_registers(mb,250,1,MB_Register))
       printf(" Version: %d.%02d ",MB_Register[0]>>8, MB_Register[0]&0xff);
     if(modbus_read_input_registers(mb,0x1000,1,MB_Register))
       if(MB_Register[0]>0)
          printf(" VDD: %5.2fV ",2.048*1023.0/MB_Register[0]);
     printf("\n");

   for(_io=0;_io<IoCount;_io++)
   {

     printf("%12s IO%d :"," ",_io);

     // read io configuratioe
     if(modbus_read_registers(mb,0x100+_io,1,MB_Register)<0)
       {
         printf(" Unable to read configuration\n");
         continue;
       }

     switch(MB_Register[0])
      {
       case IOCONFIG_ANALOGVDD: RawData(mb,_io,"ANALOG Vref=VDD",1);break;
       case IOCONFIG_ANALOG1V:  //ReadTMP35(mb,_io);
                                RawData(mb,_io,"ANALOG Vref=1.024V",1);break;
       case IOCONFIG_ANALOG2V:  RawData(mb,_io,"ANALOG Vref=2.048V",1);break;
       case IOCONFIG_ANALOG4V:  RawData(mb,_io,"ANALOG Vref=4.096V",1);break;

       case IOCONFIG_INPUT:     
       case IOCONFIG_INPUT_PULLUP:
       case IOCONFIG_OUTPUT:     RawData(mb,_io,"DIGITAL",1);break;

       case IOCONFIG_CAP_SENSE_OFF:
       case IOCONFIG_CAP_SENSE_LOW:
       case IOCONFIG_CAP_SENSE_MEDIUM:
       case IOCONFIG_CAP_SENSE_HIGH:  RawData(mb,_io,"CAP SENSE",2);break;


       case IOCONFIG_DHT11:       RawData(mb,_io,"DHT11",3);break;
       case IOCONFIG_DHT22:       ReadDHT22(mb,_io);break;
       case IOCONFIG_DS18B20:     ReadDS18B20(mb,_io);break;
       case IOCONFIG_PWM:         RegisterData(mb,_io,"PWM");break;
       case IOCONFIG_SERVO:       RegisterData(mb,_io,"R/C SERVO");break;
       case IOCONFIG_COUNTER:    RawData(mb,_io,"COUNTER",3);break;
       default:   printf("\n");
      }

 }

}




int main(int argc, char * argv[])
{
  time_t now;
  struct tm tmnow;
  int module,io;
  uint16_t  MB_Register[3];



   modbus_t *mb;

   mb = modbus_new_rtu("/dev/ttyAMA0",57600,'N',8,1);
   modbus_connect(mb);


   // Fset time out to 1/100 of second
   struct timeval response;
   response.tv_sec=0;
   response.tv_usec=7000;
   modbus_set_response_timeout(mb, &response);


    for(;;)
   {
       now = time(NULL);
       tmnow = *localtime(&now);

       printf("%2d:%02d:%02d\n",
       tmnow.tm_hour,tmnow.tm_min,tmnow.tm_sec);



       for(module=1;module<128;module++)
         {
           // check for release version
           modbus_set_slave(mb,module);
           if(modbus_read_registers(mb,251,1,MB_Register)<0)
              continue;

           PrintModule(mb,module);

         }


  }

 modbus_close(mb);
 modbus_free(mb);
 return 0;
}
