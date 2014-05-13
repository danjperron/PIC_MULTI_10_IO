
#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DS18B20.h"
#include "IOConfig.h"
#include "CAPSENSE.h"


bit  WaitForStartDeciSecond;
bit  WaitForEndDeciSecond;
bit  GotCapSenseFlag;


void  CAPSENSECycleIdle(void)
{
    unsigned char _temp;
   CurrentIOStatus=IO_STATUS_OK;
   CurrentIOCycle=IO_CYCLE_START;


   // select capsense
   CPSCON1 = CSMASK[CurrentIOPin];
   CPSON=0;
   TMR0IE=0;
   _TMR0_MSB=0;
   TMR0=0;
   WaitForStartDeciSecond=0;
   WaitForEndDeciSecond=0;
   GotCapSenseFlag=0;
   // force TMR0 TO CLOCK CAPSENSE
   if(CurrentIOSensor.Config == IOCONFIG_CAP_SENSE_OFF)
       CPSCON0= 0b10000001;
   else if(CurrentIOSensor.Config == IOCONFIG_CAP_SENSE_LOW)
       CPSCON0= 0b10000101;
   else if(CurrentIOSensor.Config == IOCONFIG_CAP_SENSE_MEDIUM)
       CPSCON0= 0b10001001;
   //if(CurrentIOSensor == IOCONFIG_CAP_SENSE_HIGH)
   else
      CPSCON0= 0b10001101;
   OPTION_REG=0B00100000;
   TMR0CS=1;
   TMR0IE=0;
   Timer0Overflow=1;
   WaitForStartDeciSecond=1;
   GotCapSenseFlag=0;
}


void DoCAPSENSECycle(void)
{
  switch(CurrentIOCycle)
  {
 
    case IO_CYCLE_IDLE:
                          CAPSENSECycleIdle();
                          break;
         case IO_CYCLE_START: if(GotCapSenseFlag)
                         {
                           
                          WorkingSensorData.BYTE[0]=0;
                          WorkingSensorData.BYTE[1]= _TMR0_MSB>>8;
                          WorkingSensorData.BYTE[2]= (_TMR0_MSB & 0xff);
                          WorkingSensorData.BYTE[3]= _TMR0;
                          _TMR0_MSB=0;
                          _TMR0=0;
                          CurrentIOCycle=IO_CYCLE_END;
                          CurrentIOStatus= IO_STATUS_OK;
                          CPSON=0;
                          TMR0CS=0;
                          TMR0IE=0;

                         }
                         break;
  }

}
