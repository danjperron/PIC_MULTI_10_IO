
#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif



#include "IOConfig.h"
#include "IOCycle.h"
#include "DS18B20.h"
#include "DHT22.h"
#include "CAPSENSE.h"


unsigned char CurrentIOStatus;
near ConfigUnion CurrentIOSensor;
near unsigned char CurrentIOPin;
near unsigned char CurrentIOCycle;

SensorDataUnion  IOSensorData[INPUT_COUNT] @ 0x1A0;
SensorDataUnion  WorkingSensorData;




volatile unsigned char _TMR0;
volatile unsigned short _TMR0_MSB;
volatile unsigned char BitCount;
volatile unsigned char WorkingByte;
volatile unsigned char CSum;
volatile unsigned char WorkingCount;
volatile unsigned  char ByteIndex;
volatile unsigned char PreBitCount;
bit      Timer0Overflow;
bit      TimerSecFlag;
near volatile unsigned char    DHTFlag           @ 0x070;
near volatile unsigned char    IOCounterFlag     @ 0x071;

// Counter will be on same bank
// than IOSensor data (bank 3)
// IOSensor is 10 data of 6 BYTE

unsigned short COUNTER0 @ 0x1E6;
unsigned short COUNTER1 @ 0x1E8;
unsigned short COUNTER2 @ 0x1EA;
unsigned short COUNTER3 @ 0x1Ec;
unsigned short COUNTER4 @ 0x1EE;


void DealWithError(void)
{
   TMR0IE=0;
   TMR0IF=0;
   WorkingSensorData.WORD[2]=0xFFFF;
   if(CurrentIOSensor.DHT)
      DHT22Error();
}


void ScanNextIOPin(void)
{
  unsigned char loop;
  CurrentIOPin++;

  if(CurrentIOPin >= INPUT_COUNT)
   CurrentIOPin=0;

  CurrentIOSensor .Config= Setting.IOConfig[CurrentIOPin];
  CurrentIOCycle= IO_CYCLE_IDLE;
  CurrentIOStatus=IO_STATUS_UNKNOWN;
  Timerms=0;
  TMR0IE=0;
  Timer0Overflow=0;

  if(CurrentIOSensor.CAP_SENSE)
      Timer0Overflow=1;

  for(loop=0;loop<SENSOR_DATA_BYTE_MAX;loop++)
      WorkingSensorData.BYTE[loop]=0;

}

void DoIOCycle(void)
{
 

 SensorDataUnion *SensorPt;

  unsigned char  loop;

  if(CurrentIOCycle==  IO_CYCLE_END)
  {
    if(CurrentIOStatus==IO_STATUS_UNKNOWN)
       DealWithError();

    // Re-organize DATA

    SensorPt = &IOSensorData[CurrentIOPin];

    // DHT22 Nothing to do data is correct

    
    if(CurrentIOStatus==IO_STATUS_BAD)
        SensorPt->WORD[0]=0xffff;
    else
    {
        SensorPt->BYTE[0]=0;
        SensorPt->BYTE[1]=CurrentIOStatus;
    }
    
    if(CurrentIOSensor.Config == IOCONFIG_DHT11)
    {
       // OK device is only 8 bits so move MSB to LSB and zero MSB
//       SensorPt->WORD[0]= CurrentIOStatus;
       SensorPt->BYTE[3]=WorkingSensorData.BYTE[0];
       SensorPt->BYTE[5]=WorkingSensorData.BYTE[2];
       SensorPt->BYTE[2]=0;
       SensorPt->BYTE[4]=0;
    }
    else if(CurrentIOSensor.Config == IOCONFIG_DHT22)
    {
       // OK device is only 8 bits so move MSB to LSB and zero MSB
//       SensorPt->WORD[0]= CurrentIOStatus;
       SensorPt->WORD[1]=WorkingSensorData.WORD[0];
       SensorPt->WORD[2]=WorkingSensorData.WORD[1];
    }
    if(CurrentIOSensor.DS18B20)
    {
        // Reorganize BYTE
//       SensorPt->WORD[0]= CurrentIOStatus;
       
       SensorPt->BYTE[2]=WorkingSensorData.BYTE[1];
       SensorPt->BYTE[3]=WorkingSensorData.BYTE[0];
       SensorPt->BYTE[4]=0;
       SensorPt->BYTE[5]=WorkingSensorData.BYTE[4];
    }

    if(CurrentIOSensor.CAP_SENSE)
    {
        // CAPSENSE
        SensorPt->DWORD=WorkingSensorData.DWORD;
    }

    // now it is time for next sensor;
         ScanNextIOPin();
 

 }
  else
// if(CurrentIOCycle < IO_CYCLE_WAIT)
  {
    if(CurrentIOSensor.DHT)
      DoDHT22Cycle();
    else if(CurrentIOSensor.DS18B20)
        DoDS18B20Cycle();
    else if(CurrentIOSensor.CAP_SENSE)
        DoCAPSENSECycle();
    else
     {
        // sensor doesn't need cycle  jump to the next one
        ScanNextIOPin();
     }
   }

}


void ResetIOCycle(void)
{
 unsigned char loop;
    
 for(loop=0;loop<INPUT_COUNT;loop++)
 {
    if((Setting.IOConfig[loop]==IOCONFIG_DHT11) ||
       (Setting.IOConfig[loop]==IOCONFIG_DHT22))
        SetIOChange(loop,0);

 CurrentIOCycle= IO_CYCLE_IDLE;
 CurrentIOSensor.Config = Setting.IOConfig[0];
 CurrentIOPin=0;
 DHTFlag=0;
}
}

void SetInputMode(unsigned char Pin)
{
 unsigned char _tmp= IOMASK[Pin];
 if(Pin<5)
     TRISB |= _tmp;
 else
     TRISA |= _tmp;
}

void SetOutputMode(unsigned char Pin)
{

 unsigned char _tmp= NOT_IOMASK[Pin];
 if(Pin<5)
     TRISB &= _tmp;
 else
     TRISA &= _tmp;


}



char  ReadIOPin(unsigned char Pin)
{
    unsigned char _tempb;
    unsigned char mask = IOMASK[Pin];
      if(Pin<5)
          _tempb = PORTB & mask;
      else
          _tempb = PORTA & mask;
      if(_tempb==0)
          return 0;
      return 1;
}

void WriteIO(unsigned char Pin,unsigned char value)
{

    unsigned char mask = IOMASK[Pin];
    unsigned char nmask = NOT_IOMASK[Pin];

    if(Pin <5)
    {
       if(value==0)
       {
           di();
           PORTB &= nmask;
           ei();
       }
       else
       {
           di();
           PORTB |= mask;
           ei();
       }

    }
    else
    {
       if(value==0)
       {
           di();
           PORTA &= nmask;
           ei();
       }
       else
       {
           di();
           PORTA |= mask;
           ei();
       }

    }


}


void SetIOChange(unsigned char Pin, unsigned char value)
{
    if(Pin<5)
        if(value==0)
        {
            di();
            IOCBN&= NOT_IOMASK[Pin];
            ei();
        }
         else
         {
            di();
            IOCBN|= IOMASK[Pin];
            ei();
         }
}




