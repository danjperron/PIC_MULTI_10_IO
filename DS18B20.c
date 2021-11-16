

#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DS18B20.h"
#include "IOConfig.h"


void DS18B20CycleIdle(void)
{
 unsigned short _temp;

// if(Timerms > 1000)  //more than 1 second
 {
   // ok Half second pass 
   // time to start pulse

   SetOutputMode(CurrentIOPin);
//   SetIOChange(CurrentIOPin,0);
  CurrentIOCycle= IO_CYCLE_START;
 }
}

near bit NodeviceFound;

void SetTimer6Delay500ns(unsigned char value)
{

 T6CON= 0b00000101;
 PR6 = value;
 TMR6=0;
 TMR6IF=0;
}

void SetTimer6Delay8us(unsigned char value)
{
    T6CON = 0b00000111;
    PR6=value;
    TMR6 =0;
    TMR6IF=0;
}



void  DS18B20Reset()
{

    SetOutputMode(CurrentIOPin);
    WriteIO(CurrentIOPin,0);
    // reset for >480 < 960 us
    // so it will be ~500us = 504
    SetTimer6Delay8us(63);
}


      
   

unsigned char DS18B20ResetCheckForDevice()
{

    SetInputMode(CurrentIOPin);

   // delay of 60us by ds18b20 follow by presence (60..240us)
   // so let's check for 90 us
   SetTimer6Delay500ns(180);
   while(!TMR6IF);

   NodeviceFound=ReadIOPin(CurrentIOPin);
    if(NodeviceFound)
       return( 0);

    // need a minimum of 480 us after reset pulse
    // than we will put 500us-90us(presence detection) = 410 (408)
    SetTimer6Delay8us(51);
       return(1);
}    
     

unsigned char DS18B20Read()
{
   unsigned char mask=1;
   unsigned char loop;
   unsigned char result=0;

    for(loop=8;loop > 0 ;loop--)
    {

     SetOutputMode(CurrentIOPin);
     WriteIO(CurrentIOPin,0);
 
     GIE=0;

 // delay 0.75 us the return from write io and set inputmode should do the time
#asm
     nop
     nop
     nop
     nop
     nop
     nop
     nop
     nop
 #endasm

     SetInputMode(CurrentIOPin);

#asm
     nop
#endasm
   // delay 1us
   SetTimer6Delay500ns(2);
   while(!TMR6IF);
   if(ReadIOPin(CurrentIOPin)>0)
   {
       GIE=1;
       result |= mask;
   }
   else
   GIE=1;
      // delay 60-1-5 = 54 us
     SetTimer6Delay500ns(120);
     while(!TMR6IF);
     mask <<=1;
    }        
  return result;
}

void DS18B20Write(unsigned char value)
{
  unsigned char loop;
  unsigned char mask=1;
  for(loop=8;loop>0;loop--)
  {
      GIE=0;
      SetOutputMode(CurrentIOPin);
      WriteIO(CurrentIOPin,0);
  
     if(value & mask)
     {
//       SetTimer6Delay500ns(2);
//       while(!TMR6IF);
#asm
       nop
       nop
#endasm
       GIE=1;
     }
     else
     {
         GIE=1;
 // delay of 60us
   SetTimer6Delay500ns(120);
   while(!TMR6IF);
     }
      
      SetInputMode(CurrentIOPin);

    if(value & mask)
    {
       //__delay_us(60);
       SetTimer6Delay500ns(120);
       while(!TMR6IF);
    }

     
    mask <<=1;
  }
}


void DoDS18B20Cycle(void)
{
 static unsigned short waitLoop;

  switch(CurrentIOCycle)
  {
    case IO_CYCLE_IDLE: 
                          DS18B20CycleIdle();
                          return;
    case IO_CYCLE_START:
                         CurrentIOCycle = IO_CYCLE_DS18B20_START;
    case IO_CYCLE_DS18B20_RESET2:
                         DS18B20Reset();
                         break;
    case IO_CYCLE_DS18B20_RESET_WAIT:  //wait ~500us
    case IO_CYCLE_DS18B20_RESET2_WAIT:
      case IO_CYCLE_DS18B20_RESET_WAIT2:
      case IO_CYCLE_DS18B20_RESET2_WAIT2:
        if(TMR6IF)
            break; //ok time out next
        return;

    case IO_CYCLE_DS18B20_RESET_CHECK_FOR_DEVICE:
    case IO_CYCLE_DS18B20_RESET2_CHECK_FOR_DEVICE:
          if(DS18B20ResetCheckForDevice()==0)
                        {
                         CurrentIOStatus=IO_STATUS_BAD;
                         CurrentIOCycle=IO_CYCLE_END;
                         return;
                        }
                        
                        break;
    case IO_CYCLE_DS18B20_SKIP_ROM:
    case IO_CYCLE_DS18B20_SKIP_ROM2:
                        DS18B20Write(DS18B20_SKIP_ROM);
                        break;
    case IO_CYCLE_DS18B20_CONVERT_T:
                        DS18B20Write(DS18B20_CONVERT_T);
                        waitLoop=600;
                        // set 2 ms delay 8us * 250 = 2 ms
                        SetTimer6Delay8us(250);
                        break;
    case IO_CYCLE_DS18B20_WAIT:// wait ~800 ms for conversion
                        if(!TMR6IF)  // is 8ms timeout  from timer0 done
                           return;
                        waitLoop--;
                        if(waitLoop==0)
                          break;
                        // set 2 ms delay
                        SetTimer6Delay8us(250);
                        return;
     case IO_CYCLE_DS18B20_READ_SCRATCHPAD:
                        DS18B20Write(DS18B20_READ_SCRATCHPAD);
                        ByteIndex=0;
		        break;
    case IO_CYCLE_DS18B20_READ_BYTE:

                        WorkingSensorData.BYTE[ByteIndex]=DS18B20Read();
                        ByteIndex++;
                        if(ByteIndex>5)
                           {
                            CurrentIOCycle=IO_CYCLE_END;
                            CurrentIOStatus=IO_STATUS_OK;
                           }
                       return;           
  } 

   CurrentIOCycle++;    
}



