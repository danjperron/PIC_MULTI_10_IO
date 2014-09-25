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
near unsigned char ServoMask;
void DoRCServo(void)
{
    unsigned char BMask;
    short _temp;

    if(!TMR1ON)
    {
        ServoIndex++;
    if(ServoIndex>INPUT_COUNT)
    {
       ServoIndex=0;
       ServoIdleTime=20000;
    }
//     if(Setting.IOConfig[CurrentIOPin]!=IOCONFIG_SERVO)
//     {
//         ServoMask=0xff;
//         return;
//    }
     

        ServoMask=NOT_IOMASK[ServoIndex];

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
     BMask = IOMASK[ServoIndex];
     if(ServoIndex<5)
    {

        di();
//        PORTB |= BMask;
#asm
        movf DoRCServo@BMask,w
        iorwf 13,f
#endasm
    }
    else
    {
        di();
//        PORTA |= BMask;
#asm
        movf DoRCServo@BMask,w
        iorwf 12,f
#endasm
    }
    TMR1ON=1;
    ei();
    }

}