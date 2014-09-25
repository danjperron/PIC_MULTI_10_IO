#ifndef IOCYCLE
#define IOCYCLE



#include "IOCONFIG.h"

#define IO_CYCLE_IDLE          0
#define IO_CYCLE_START         1


#define IO_CYCLE_WAIT          98
#define IO_CYCLE_END           99
extern near volatile unsigned short Timerms;
extern near ConfigUnion CurrentIOSensor;
extern near unsigned char CurrentIOPin;
extern near unsigned char CurrentIOCycle;
extern unsigned char CurrentIOStatus;
extern near unsigned char Retry;

extern  bit      Timer0Overflow;
extern  bit      TimerSecFlag;
extern  bit      ResetCounterFlag;

extern near volatile unsigned char    DHTFlag     @ 0x070;
typedef union {
    struct {
        unsigned NO_USED                :3;
        unsigned IO0               :1;
        unsigned IO1               :1;
        unsigned IO2               :1;
        unsigned IO3               :1;
        unsigned IO4               :1;
    };
    unsigned char Byte;
} IOBits_t;

extern volatile IOBits_t DHTFlagbits  @ 0x070;


//extern near volatile unsigned char    IOCounterFlag     @ 0x071;
extern near volatile IOBits_t IOCounterFlag  @ 0x071;
extern near volatile IOBits_t IOCounterReset @ 0x072;







#define IO_STATUS_UNKNOWN 0
#define IO_STATUS_OK      1
#define IO_STATUS_BAD     0xff


#define SENSOR_DATA_MAX 3
#define SENSOR_DATA_BYTE_MAX (SENSOR_DATA_MAX * 2)


typedef union {
    unsigned char BYTE[SENSOR_DATA_BYTE_MAX];
    unsigned short WORD[SENSOR_DATA_MAX];
    unsigned long  DWORD;
}SensorDataUnion; 


// ICOUNTER and IOSensorData are on bank3
// ARRAYx  use in assembly mode
#define ARRAY0 0
#define ARRAY1 (SENSOR_DATA_BYTE_MAX)
#define ARRAY2 (2*(SENSOR_DATA_BYTE_MAX))
#define ARRAY3 (3*(SENSOR_DATA_BYTE_MAX))
#define ARRAY4 (4*(SENSOR_DATA_BYTE_MAX))



extern SensorDataUnion  WorkingSensorData;
extern SensorDataUnion  IOSensorData[INPUT_COUNT] @ 0x1A0;
extern unsigned char  ICOUNTER[5]  @ 0x1E0;
extern unsigned short COUNTER[5]   @ 0x1E6;


extern void DoIOCycle(void);
//extern void DealWithIOCBF(void );
extern void ResetIOCycle(void);
extern void SetIOChange(unsigned char Pin, unsigned char value);
extern void SetInputMode(unsigned char Pin);
extern void SetOutputMode(unsigned char Pin);
extern unsigned char ReadIOPin(unsigned char Pin);
extern void  WriteIO(unsigned char Pin,unsigned char value);

extern volatile unsigned char _TMR0;
extern volatile unsigned short _TMR0_MSB;
extern volatile unsigned char BitCount;
extern volatile unsigned char WorkingByte;
extern volatile unsigned char CSum;
extern volatile unsigned  char ByteIndex;


#endif