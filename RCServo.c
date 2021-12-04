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
    unsigned short _temp;

    if(!TMR1ON)
    {

//        ServoIndex++;
    {
#asm
       incf _ServoIndex,f 
#endasm
    }    
        
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
            TMR1= (~((unsigned short)ServoIdleTime))+1;
        }
        else
        {
            _temp = ServoTimer[ServoIndex];
            ServoIdleTime-= _temp;
            if(_temp==0)
                return;
            TMR1= (~_temp)+1;
        }

         TMR1IF=0;
         TMR1IE=1;
     BMask = IOMASK[ServoIndex];
     di();
     if(ServoIndex<5)
    {
        #ifndef USEASM
             PORTB |= BMask;
        #else
            {        
            #asm
                    movf DoRCServo@BMask,w
                    iorwf 13,f
            #endasm
            }
        #endif
    }
    else
    {
        #ifndef USEASM
            PORTA |= BMask;
        #else
            {
            #asm
                movf DoRCServo@BMask,w
                iorwf 12,f
            #endasm
            }
        #endif
    }
    TMR1ON=1;
    ei();
    }
}