
#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DHT22.h"
#include "IOConfig.h"

volatile unsigned char _TMR0;

volatile unsigned char BitCount;
volatile unsigned char WorkingByte;
volatile unsigned char CSum;
volatile unsigned char WorkingCount;
volatile unsigned  char ByteIndex;
volatile unsigned char PreBitCount;



void DHT22Error(void)
{
int loop;
 // KILL interrup
TMR0IE=0;
SetIOChange(CurrentIOPin,0);
CurrentIOCycle=IO_CYCLE_END;
SetInputMode(CurrentIOPin);
}





void DHT22CycleIdle(void)
{
 unsigned short _temp;
 unsigned char _ctemp;

if(Timerms > 1000)  // more than 1000ms
 {
   // ok Half second pass 
   // time to start pulse
 _ctemp = NOT_IOMASK[CurrentIOPin];

      di();
      IOCBN &= _ctemp;
      IOCBF &= _ctemp;
      TRISB &= _ctemp;
      PORTB &= _ctemp;
      ei();

    CurrentIOCycle= IO_CYCLE_START;
    BitCount=40;
    WorkingCount=8;
    WorkingByte=0;
    ByteIndex=0;
    PreBitCount=2;
    // set timer0 to 1 us clock
// assume 32Mhz
 OPTION_REG	= 0b00000010;	// pullups on, TMR0 @ Fosc/4/8

    TMR0=0;
    CSum=0;
 }
}

void DHT22CycleStart(void)
{
    unsigned char _ctemp= IOMASK[CurrentIOPin];
    unsigned char _ntemp= ~_ctemp;
  // wait until Timerms got at least 2ms (2 counts).
  if(Timerms >2)
  {
    // release for

   
      TRISB |= _ctemp;
   

  // wait a little
     asm("NOP");
   

  

  TMR0=0; // reset Timer 0

  DHTFlag=_ctemp; // set bit to correct DHT22 Cycle
  di();
  IOCBF &= _ntemp;
  IOCBN |= _ctemp;
  ei();

  IOCIE=1;

// enable TMR0 interrupt
TMR0IF = 0;
TMR0IE = 1;
CurrentIOStatus=IO_STATUS_UNKNOWN;

  CurrentIOCycle= IO_CYCLE_WAIT;
}
}


void DoDHT22Cycle(void)
{ 
  switch(CurrentIOCycle)
  {
   case IO_CYCLE_WAIT:  break; // do nothing
   case IO_CYCLE_END:   break; // do nothing
   case IO_CYCLE_START: DHT22CycleStart();break;
   default: DHT22CycleIdle();
 }
}

 
 void DHT22IOCBF(void)
{
 

 if(PreBitCount >0)
     PreBitCount--;
else 
 {


   WorkingByte*=2;
   if(_TMR0 > 100)
   WorkingByte|=1;
   WorkingCount--;
   if(WorkingCount==0)
   {
      WorkingCount=8;
      if(ByteIndex < 4)
      {
        CSum += WorkingByte;
        WorkingSensorData.BYTE[ByteIndex++]=WorkingByte;
      }

   }
   BitCount--;
   if(BitCount==0){

    // is Check Sum OK
   if(CSum == WorkingByte)
       CurrentIOStatus=IO_STATUS_OK;
      else
       CurrentIOStatus=IO_STATUS_BAD;

   SetIOChange(CurrentIOPin,0);
   TMR0IE=0;
   CurrentIOCycle=IO_CYCLE_END;
}
}
  

}
