
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
volatile unsigned  char ByteIndex;

// use bank 4 to store info

unsigned char DHTBitBuffer[47] @ 0x220;
unsigned char DHTBufferIndex @ 0x24f;


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
      ei();
      di();
      TRISB &= _ctemp;
      PORTB &= _ctemp;
      ei();

    CurrentIOCycle= IO_CYCLE_START;
    BitCount=40;
    WorkingByte=0;
    ByteIndex=0;
    // set timer0 to 1 us clock
// assume 32Mhz
 OPTION_REG	= 0b00000010;	// pullups on, TMR0 @ Fosc/4/8

    TMR0=0;

 }
}

void DHT22CycleStart(void)
{
    unsigned char _ctemp= IOMASK[CurrentIOPin];
    unsigned char _ntemp= ~_ctemp;
  // wait until Timerms got at least 2ms (2 counts).

CurrentIOStatus=IO_STATUS_UNKNOWN;
CurrentIOCycle= IO_CYCLE_WAIT;

    if(Timerms >3)
  {
    // release for

   
   

   di();
  IOCBF &= _ntemp;
  IOCBN |= _ctemp;
  ei();
  DHTFlag=_ctemp; // set bit to correct DHT22 Cycle

  DHTBufferIndex=0;;

  IOCIE=1;


  TMR0=0; // reset Timer 0
  TMR0IF = 0;
  TMR0IE = 1;

  di();
  TMR0=0;
  TRISB |= _ctemp;
  ei();

  // wait a little
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");



// enable TMR0 interrupt
}
}


void DoDHT22Cycle(void)
{
    if(CurrentIOCycle==IO_CYCLE_WAIT)
        return;
    else if(CurrentIOCycle == IO_CYCLE_DHT_ANALYZE)
        CheckDHTBitBuffer();
    else if(CurrentIOCycle == IO_CYCLE_END)
        return;
    else if(CurrentIOCycle == IO_CYCLE_START)
        DHT22CycleStart();
    else
        DHT22CycleIdle();
}
 
void CheckDHTBitBuffer(void)
{
    unsigned char loop,loop2;
    TMR0IE=0;
    SetIOChange(CurrentIOPin,0);
    CurrentIOStatus=IO_STATUS_BAD;
    if(DHTBufferIndex>=42)
    {
    WorkingByte=0;
    BitCount=2;
    CSum=0;

    for(loop=0;loop<5;loop++)
    {
        for(loop2=0;loop2<8;loop2++)
        {
            WorkingByte*=2;
            if(DHTBitBuffer[BitCount++] > 110)
            WorkingByte++;
        }
        if(loop<4)
        {
         CSum += WorkingByte;
         WorkingSensorData.BYTE[loop]=WorkingByte;
        }
    }

    if(CSum == WorkingByte)
    {
        CurrentIOStatus = IO_STATUS_OK;
        CurrentIOCycle= IO_CYCLE_END;
        return;
    }
    }

    if(Retry==0)
    {
        Retry++;
        Timerms=0;
        CurrentIOCycle=IO_CYCLE_IDLE;
    }
    else
    CurrentIOCycle=IO_CYCLE_END;
}