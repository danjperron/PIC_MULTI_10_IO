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
extern  bit      Timer0Overflow;
extern  bit      TimerSecFlag;

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
} IOBits_t;

extern volatile IOBits_t DHTFlagbits  @ 0x070;


extern near volatile unsigned char    IOCounterFlag     @ 0x071;
extern volatile IOBits_t IOCounterFlagbits  @ 0x071;


extern unsigned short COUNTER0;
extern unsigned short COUNTER1;
extern unsigned short COUNTER2;
extern unsigned short COUNTER3;
extern unsigned short COUNTER4;


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

extern SensorDataUnion  WorkingSensorData;
extern SensorDataUnion  IOSensorData[INPUT_COUNT] @ 0x1A0;


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
extern volatile unsigned char WorkingCount;
extern volatile unsigned  char ByteIndex;
extern volatile unsigned char PreBitCount;


#endif