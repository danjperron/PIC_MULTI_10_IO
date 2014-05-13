#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DHT22.h"
#include "IOConfig.h"
#include "RCServo.h"

unsigned char ServoIndex;
unsigned short ServoTimer[INPUT_COUNT];
short ServoIdleTime=16000;



void DoRCServo(void)
{
    short _temp;

    if(!TMR1ON)
    {
        ServoIndex++;
    if(ServoIndex>INPUT_COUNT)
    {
       ServoIndex=0;
       ServoIdleTime=16000;
    }

        if(ServoIndex==INPUT_COUNT)
        {
            if(ServoIdleTime<0)
                return;
            TMR1= (~ServoIdleTime)+1;
        }
        else
        {
            _temp = ServoTimer[ServoIndex];
            ServoIdleTime-=(short) _temp;
            if(_temp==0)
                return;
            TMR1= (~_temp)+1;
        }

         TMR1IF=0;
         TMR1IE=1;
         TMR1ON=1;
     if(ServoIndex<5)
    {
        di();
        PORTB |= IOMASK[ServoIndex];
        ei();
    }
    else
    {
        di();
        PORTA |= IOMASK[ServoIndex];
        ei();
    }

    }

}