/* 
 * File:   main.c
 * Author: daniel
 *
 * Created on 27 mars 2014, 21:59
 */

#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif

#include "IOCycle.h"
#include "DHT22.h"
#include "DS18B20.h"
#include "CAPSENSE.h"
#include "IOConfig.h"
#include "RCServo.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MultiIO10   Multifunction 10 pins
//
// Simple program with different function on each pin
// using RS-485 with modbus protocol to communicate
// 
//
//  List of IO mode
//
//
// 1- Digital input.  ( IO0..IO4 could have PULL_UP resistor)
// 2- Digital output.
// 3- Analog input withc selectable reference voltage.
//    (1.024,2.048,4.096V and  VDD).
// 4- R/C servo pwm mode . select step value in usec.
//    ( Servo use software timer).
// 5- PWM output  with 10 bits resolution (0..1023).
//     Hardware comparator at 1.9Khz cycle ferquency
// 5- Reading of DHT11,DHT22 and AMM2302 temperature sensor.
// 6- Reading of DS18B20 temperature sensor
// 7- Cap sensor signale with 4 different power setting.
// 8- Pulse counter. Give Pulse count at Pulse/sec.

//   Date: 27 Mars 2014
//   programmer: Daniel Perron
//   Version: 1.0
//   Processeur: PIC12F1840
//   logiciel de compilation:  Microchip MPLAB  X IDE (freeware version)
//
//   The system use serial communication at 57600 BAUD with modbus protocol.
//  
//   Pin RB0 control the serial communication direction.
//
//
//   MODBUS REGISTER LIST
//
//
/////  Modbus Function  01 & 02
//
//  Lire IO0 to IO9
//
//  Modbus [SLA][1][0][A][0][1][CRC]
//
//  SLA - Adresse
//  A   - 0=IO0 1=IO1 ... 9=IO9
//  CRC - Cyclical redundancy check.
//
//  Ex:   Python Read IO0 Bit on slaveAddress module 1 (using PicModule.py)
//
//    import PicModule
//    m1 = PicModule.PicMbus(1)
//    m1.readIO(0)
//
//
//
/////  Modbus Function 3
//
//  Modbus [SLA][3][0][A][0][1][CRC]
//
//  A =
//    0..9 :  Read  R/C servo or PWM value of IO
//     160 :  Read slave modbus address
//     250 :  Read Software version
//     251 :  Read Software Id number
// (0x100 .. 0x109):   Read IO configuration Mode
// 
//
//
// Ex:  Python
//   Lire numéro d'identification du logiciel
//   m1.readId():
//
//   or  the long way
//
//   m1.module.read_register(251,0,3);
//
//
///// Modbus Fonction 4
//
//  Modbus [SLA][4][0][A][0][Number of register][crc]
//

  // Function 4  Read Current Register
  //
  // Address 0x1000:  Current VRef 2.048V A/D value
  // Address 0x1001:  Current Build-in Temperature Sensor
  // Address 0xn0:  Read current IOn
//  A =  0x00N0:  Read IO Sensor data where N is the IO id number.
//       0x1000:  Read the 2.048V ref with the VDD has reference.
//       0x1001:  Read build-in temperature diode. (VDD=Vref).
//
//
//  Ex: Read DS18B20 Sensor At IO2
//
//     m1.readDS18B20(2)
//
//
//
///// Modbus Fonction 5
//
//  Modbus [SLA][5][0][A][0][DATA][crc]
//
//  A = 0..9 : Set IO bit output
//
//
///// Modbus Fonction 6
//
//  Modbus [SLA][6][0][A][16 bits DATA][CRC]
//
//  A = 0..9:  Set R/C Servo or PWM digital output
//             Also clear COUNTER (In counter Mode)
//       160:  Set new modbus slave address
// 0x100..0x10a:  Set new IO configuration  (IOCONFIG.H)

//
//  ex:    Change IO0 configuration to read DHT22 SENSOR
//  
//   m1.config(0,m1.IOCONFIG_DHT22)
// 
//


#define SOFTWARE_ID      0x653A
#define RELEASE_VERSION 0x0100

///////////////////  How to program the IC.
//
//    1 - Use MPLAB with pickit 3  (need version 8.90 at least)
//  or
//    2 - Use standalone pickit2 v 2.61 (Select the I.C. and load the corresponding hex file).
//  or
//    3 - Use Raspberry Pi board  with burnVLP.py . Software available from https://github.com/danjperron/burnLVP
//
////////////////////////////////////  GPL LICENSE ///////////////////////////////////


/*
 This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



/*  COMMUNICATION PROTOCOL

     baud rate 57600 , 8 bits data , no parity
    

//////////////////////////  PORT DESCRIPTION
/*
 * IO5 RA0
 * IO6 RA1
 * IO7 RA2
 * IO8 RA3
 * IO9 RA4
 *     RA5    MCLR (RESET)
 *     RA6    CRYSTAL CLOCK
 *     RA7    CRYSTAL CLOCK
 *     RB0    RS-485 DIRECTION
 *     RB1    SERIAL IN
 *     RB2    SERIAL OUT
 * IO0 RB3
 * IO1 RB4
 * IO2 RB5
 * IO3 RB6
 * IO4 RB7
 *
 * P.S. ONLY IO0..IO4 support Counter, Pull-up, Cap Sense and DHT type sensor.
 *
 *
*/



//rs-485 data direction

#define TXM_ENABLE   RB0



#ifndef BORV_LO
#define BORV_LO BORV_19
#endif

#define iabs(A)  (A<0 ?  (-A) :  A)


// CONFIG1

#ifdef USE_EXTERNAL_XTAL
    #pragma config FOSC= HS
#else
    #pragma config FOSC = INTOSC // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#endif
#pragma config WDTE = ON // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF // Flash Memory Self-Write Protection (Write protection off)
   
#ifdef USE_EXTERNAL_XTAL
  #pragma config PLLEN = ON // PLL Enable (4x PLL enabled)
#else
  #pragma config PLLEN = OFF // PLL Enable (4x PLL enabled)
#endif
#pragma config STVREN = ON // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON // Low-Voltage Programming Enable (Low-voltage programming enabled)



//Set default value
//  IO0..IO9 config , modbus address
__EEPROM_DATA(IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT,IOCONFIG_INPUT);
__EEPROM_DATA(IOCONFIG_INPUT,IOCONFIG_INPUT,127,0xff,0xff,0xff,0xff,0xff);


unsigned char VRange;  //1 = 1.024V,  2 = 2.048V, 3 = 4.096V else = VDD

unsigned char BadIO; 
SettingStruct Setting;

#define TIMER_100MS  100
near volatile unsigned short Timerms;       //  Interrupt Timer counter in 1 ms
near volatile unsigned short PrimaryTimerms;
near volatile unsigned char TimerDeciSec;    // modbus timer out in 1/10 of sec  in 0.5 ms count
#pragma pack 1
typedef union {
  unsigned short USHORT;
  unsigned char BYTE[2];
}ByteShortUnion;


//near unsigned char CurrentTimer1H;
//near unsigned char CurrentTimer1L;

// serial buffer
#define SERIAL_BUFFER_SIZE 32
near volatile unsigned char InFiFo, OutFiFo;   // these are the buffer pointers for interrupt serial communication; InFiFo ctrl by putch, OutFiFo ctrl by interrupt
near volatile unsigned char RcvInFiFo, RcvOutFiFo;
char SerialBuffer[SERIAL_BUFFER_SIZE];
char RcvSerialBuffer[SERIAL_BUFFER_SIZE];

unsigned char SerialSum;    // use for check sum
unsigned char RcvSerialSum;  // use for check sum verification
bit ModbusOnTransmit;
bit ForceReset;
bit EnableConfigChange;
char ModbusPacketBuffer[SERIAL_BUFFER_SIZE] @ 0x220;


const unsigned char     IOMASK[11]={0b00001000,0b00010000,0b00100000,0b01000000,0b10000000,\
                                    0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0};

const unsigned char NOT_IOMASK[11]={0b11110111,0b11101111,0b11011111,0b10111111,0b01111111,\
                                    0b11111110,0b11111101,0b11111011,0b11110111,0b11101111,0b11111111};

//cap sense mask
const unsigned char CSMASK[10]={0b00001001,0b00001000,0b00000111,0b00000101,0b00000110,\
                                0b00000000,0b00000001,0b00000010,0b00000011,0b00000100};

// MODBUS


// CRC16 source code is in CRC16.c file
extern unsigned short CRC16(unsigned char * puchMsg, unsigned char usDataLen);


unsigned char ModbusFunction;
unsigned char ModbusSlave;
unsigned short ModbusAddress;
unsigned short ModbusData;
volatile unsigned short ModbusCRC;
unsigned char ModbusFramePointer;

unsigned char ModbusBuffer[10];

//MODBUS EXCEPTION
#define ILLEGAL_FUNCTION     1
#define ILLEGAL_DATA_ADDRESS 2
#define ILLEGAL_DATA_VALUE   3
#define SLAVE_DEVICE_FAILURE 4
#define ACKNOWLEDGE          5
#define SLAVE_DEVICE_BUZY    6
#define NEGATIVE_AKNOWLEDGE  7
#define MEMORY_PARITY_ERROR  8


/* Timer utilisation
Timer0  interrupt timer
Timer1  Sensor timer  utility, 
Timer2  Servo   Utility
Timer4  Hardware PWM
*/





  


// EEPROM LOAD AND SAVE SETTING

void LoadSetting(void)
{
  unsigned char  idx;
  unsigned char  * pointer = (unsigned char *) &Setting;

  for(idx=0; idx < sizeof(Setting);idx++)
     *(pointer++) = eeprom_read(idx);
}

void SaveSetting(void)
{
  unsigned char  idx;
  unsigned char  * pointer = (unsigned char *) &Setting;

  for(idx=0; idx < sizeof(Setting);idx++)
      eeprom_write(idx, *(pointer++));
}



void Init1msTimer()
{
// TIMER2
// 16Mhz clock / 4 =  250 ns clock
// 250ns * 8  = 2us clock
//  1000us / 2us = 250


     // assume 32Mhz
   T2CON= 0b00000111;
   PR2=125;


 TMR2=0;
 // Enable IRQ
TMR2IF=0;
PrimaryTimerms=100;
TimerDeciSec=10;
TMR2IE=1;
PEIE = 1;
GIE=1;
}

void InitA2D()
{
ADCON1 =  0b11100011;  // fosc/64  32Mhz
ADCON0= 0b00000001; // enable a/d
ANSELA = 0b0000000;      // NO ANALOG
ANSELB = 0b0000000;
ADIE=0;
ADIF=0;
FVRCON=0b11000010;  // Vref internal 2.048V on ADC
}

void SetAnalogConfig(unsigned char Pin)
{
// SET ANALOG PIN
 
 unsigned char ioconfig = Setting.IOConfig[Pin];
 unsigned char _tmp= IOMASK[Pin];
 if(Pin<5)
 {
     TRISB |= _tmp;
     ANSELB |= _tmp;
 }
 else
 {
     TRISA |= _tmp;
     ANSELA |= _tmp;
 }

// SET REFERENCE VOLTAGE
   
  FVRCONbits.ADFVR = ioconfig;
 
// VREF or VDD
  if(ioconfig== IOCONFIG_ANALOGVDD)
    ADCON1bits.ADPREF=0;
  else
    ADCON1bits.ADPREF=3;
}





void SetOutputConfig(unsigned char Pin)
{

    unsigned char ioconfig = Setting.IOConfig[Pin];
 unsigned char _tmp= NOT_IOMASK[Pin];
 if(Pin<5)
 {
     PORTB &= _tmp;
     TRISB &= _tmp;
     ANSELB &= _tmp;
 }
 else
 {
     PORTA &= _tmp;
     TRISA &= _tmp;
     ANSELA &= _tmp;
 }
}


char SetPWMConfig(unsigned char Pin,unsigned short value)
{

    unsigned char msb;
    unsigned char lsb;  // lsb will hold the CCPIxCON + 2  lsb bits

    msb =  value >>2;
    lsb = value & 3;
    lsb <<=4;
    lsb |= 0b00001100;
    if(Setting.IOConfig[Pin]!=IOCONFIG_PWM)
        return 0;

    if(Pin==0)
    {
        // CCP1
        TRISBbits.TRISB3=0;// output
        CCP1CON= lsb;
        CCPR1L = msb;
    }
    else if(Pin==3)
    {
        TRISBbits.TRISB6=0;// output
        CCP2CON= lsb;
        CCPR2L = msb;
    }
    else if(Pin==8)
    {
        TRISAbits.TRISA3=0;// output
        CCP3CON= lsb;
        CCPR3L = msb;
    }
    else if(Pin==9)
    {
        TRISAbits.TRISA4=0;// output
        CCP4CON= lsb;
        CCPR4L = msb;
    }
    else
    {
        BadIO=0;
        return 0;
    }
    ServoTimer[Pin]=value;
    return 1;

    

}


void SetPullUp(unsigned char Pin, unsigned char PullUp)
{
    if(Pin<5)
    {
        if(PullUp)
         WPUB |= IOMASK[Pin];
        else
         WPUB &= NOT_IOMASK[Pin];
    }
    else BadIO=1;
 
}

void SetInputConfig(unsigned char Pin)
{
    unsigned char _tmp = IOMASK[Pin];
    unsigned char _ntmp= NOT_IOMASK[Pin];
  if(Pin<5)
  {
    TRISB |= _tmp;
    ANSELB &= _ntmp;
  }
  else
  {
    TRISA |= _tmp;
    ANSELA &= _ntmp;
  }

}



void SetIOConfig(unsigned char Pin)
{
    unsigned char loop;
  ConfigUnion ioconfig;

  ioconfig.Config = Setting.IOConfig[Pin];
  ResetIOCycle();

  IOSensorData[Pin].DWORD=0;
  IOSensorData[Pin].WORD[2]=0;


  if(Pin<5)
  {
     IOCounterFlag.Byte & = NOT_IOMASK[Pin];
  }

  if(Pin==0)
  {
      CCP1CON=0;
  }
 if(Pin==3)
 {
      CCP2CON=0;
 }
  if(Pin==8)
      CCP3CON=0;
  if(Pin==9)
      CCP4CON=0;

  if(Pin<5)
  {
  SetPullUp(Pin,1);  // By default pull up is there
  SetIOChange(Pin,0);
  }
  ServoTimer[Pin]=0; // disable Servo

  if(ioconfig.Config<5)
  {
      if(Pin<5)
          SetPullUp(Pin,0);
      SetAnalogConfig(Pin);
  }
  else if((ioconfig.SERVO) || (ioconfig.Config==IOCONFIG_OUTPUT))
  {
      SetOutputConfig(Pin);
  }
  else if(ioconfig.Config == IOCONFIG_PWM)
  {
      SetPWMConfig(Pin,0);
  }
  else if(ioconfig.COUNTER)
  {
                            SetPullUp(Pin,0);
                            SetInputConfig(Pin);
                            SetIOChange(Pin,1);
                            if(Pin==0)
                                IOCounterFlag.IO0=1;
                            if(Pin==1)
                                IOCounterFlag.IO1=1;
                            if(Pin==2)
                                IOCounterFlag.IO2=1;
                            if(Pin==3)
                                IOCounterFlag.IO3=1;
                            if(Pin==4)
                                IOCounterFlag.IO4=1;
  }
  else if((ioconfig.CAP_SENSE) || (ioconfig.Config == IOCONFIG_INPUT))
  {
      if(ioconfig.CAP_SENSE)
      {
         if(Pin>4)
          {BadIO=1;return;}
      }
      if(Pin<5)
          SetPullUp(Pin,0);
      SetInputConfig(Pin);
  }
  else if(ioconfig.DHT)
  {
      if(Pin>4)
       {BadIO=1;return;}
  }
  else
    SetInputConfig(Pin);
}






unsigned short ReadA2D(unsigned char channel)
{

 ByteShortUnion value;
 ADIE=0;                              // clear interrupt flag
 ADIF=0;
 ADON=1;
 ADCON0bits.ADON=1;
 ADCON0bits.CHS=channel;
 __delay_ms(1);
 ADCON0bits.ADGO=1;
 while(ADCON0bits.ADGO==1);
__delay_ms(1);
 ADCON0bits.ADGO=1;
 while(ADCON0bits.ADGO==1);
 value.BYTE[1]=ADRESH;
 value.BYTE[0]=ADRESL;
 return value.USHORT;
}



static void interrupt isr(void){
    static volatile unsigned char _temp;


// timer 1 use by R/C Servo
    if(TMR1IE)
    if(TMR1IF)
{
    TMR1IF=0;
    if(ServoIndex<5)
    {
      //PORTB &= ServoMask
#asm
        movf _ServoMask,w
        andwf _PORTB,f
#endasm
    }
    else
    {
      // PORTA &= ServoMask
#asm
        movf _ServoMask,w
        andwf _PORTA,f
#endasm
    }
    TMR1ON=0;
  }
if(IOCIE)
if(IOCIF)
 {
//    IOCBF=0;

//      _TMR0 = TMR0;
#asm
    movf _TMR0,w
    movwf __TMR0
#endasm

      if(IOCBF & DHTFlag)
      {
          // to speed up things, reducing interrupt process ,
          // store timer 0 info into  circular buffer and
          // figure out timing calculation on main program
// use assembly (2 us faster)
//          DHTBitBuffer[DHTBufferIndex++]=_TMR0;
//          if(DHTBufferIndex<46)
//              DHTBufferIndex++;
//          TMR0=0;

#asm
   clrf _TMR0
   banksel(_DHTBufferIndex)
   movf _DHTBufferIndex^512,w
   addlw 0x20
   movwf 6
   movlw 2
   movwf 7
   movf _DHTBufferIndex^512,w
   sublw 45
   skipnc
   incf _DHTBufferIndex^512,f
   banksel(0)
   movf __TMR0,w
   movwf 1
; IOCBF&=~DHTFlag;
   comf _DHTFlag,w
   movlb 7; select bank 7
   andwf _IOCBF&0x7f,f
#endasm

 }

//}

    //  _temp = IOCBF & IOCBN;
    {
#asm
    movlb  7 ; select bank7
    movf   22,w
    andwf  21,w
    skipnz
    goto SKIPIOC
    movlb  0
    movwf isr@_temp
#endasm            
    }
    
    if(_temp&8)
    {
        //IO0
        IOCBFbits.IOCBF3=0;

        if(IOCounterFlag.IO0)
        {
//            IOSensorData[0].DWORD++;
//             COUNTER0++;
// use assembly since variable need to be   big endian
// MODBUS use BIG ENDIAN
// IOSensorData force to be bank3 so it is ^384
// IOSensorData is 6BYTE
// to reduce time on irq just increment ICOUNTER
// and then on main program  add ICOUNTER IOSensorData[0].DWORD++;
            
#asm 
        banksel(_ICOUNTER)
        incf _ICOUNTER^384,f
        btfss _IOCounterReset,3
        goto  INCCOUNTERIO0
        bcf _IOCounterReset,3
        movf (_COUNTER)^384,w
        movwf (_IOSensorData+4)^384
        movf (_COUNTER+1)^384,w
        movwf (_IOSensorData+5)^384
        clrf (_COUNTER)^384
        clrf (_COUNTER+1)^384
INCCOUNTERIO0:
        incf ((_COUNTER+1)^384),f
        skipnz
        incf (_COUNTER^384),f
        movlb 0
#endasm

        }
 
    }

      if(_temp&0x10)
    {
        //IO1
        IOCBFbits.IOCBF4=0;

        if(IOCounterFlag.IO1)
        {
//           IOSensorData[1].DWORD++;
//            COUNTER1++;
// use assembly since variable need to be   big endian
//
//
//
// to reduce time on irq just increment ICOUNTER
// and then on main program  add ICOUNTER IOSensorData[1].DWORD++;

#asm
        banksel(_ICOUNTER)
        incf (_ICOUNTER+1)^384,f
        btfss _IOCounterReset,4
        goto  INCCOUNTERIO1
        bcf _IOCounterReset,4
        movf (_COUNTER+2)^384,w
        movwf (_IOSensorData+4+ARRAY1)^384
        movf (_COUNTER+3)^384,w
        movwf (_IOSensorData+5+ARRAY1)^384
        clrf (_COUNTER+2)^384
        clrf (_COUNTER+3)^384
INCCOUNTERIO1:
        incf ((_COUNTER+3)^384),f
        skipnz
        incf ((_COUNTER+2)^384),f
        movlb 0
#endasm

        }
    }

    if(_temp&0x20)
    {
        //IO2
        IOCBFbits.IOCBF5=0;

        if(IOCounterFlag.IO2)
        {
//           IOSensorData[2].DWORD++;
//            COUNTER2++;
// use assembly since variable need to be   big endian
// to reduce time on irq just increment ICOUNTER
// and then on main program  add ICOUNTER IOSensorData[2].DWORD++;

#asm
        banksel(_ICOUNTER)
        incf (_ICOUNTER+2)^384,f
        btfss _IOCounterReset,5
        goto  INCCOUNTERIO2
        bcf _IOCounterReset,5
        movf (_COUNTER+4)^384,w
        movwf (_IOSensorData+4+ARRAY2)^384
        movf (_COUNTER+5)^384,w
        movwf (_IOSensorData+5+ARRAY2)^384
        clrf (_COUNTER+4)^384
        clrf (_COUNTER+5)^384                  
INCCOUNTERIO2:
        incf ((_COUNTER+5)^384),f
        skipnz
        incf ((_COUNTER+4)^384),f
        movlb 0
#endasm

        }
 
    }

    if(_temp&0x40)
    {
        //IO3
        IOCBFbits.IOCBF6=0;

        if(IOCounterFlag.IO3)
        {
//           IOSensorData[3].DWORD++;
//            COUNTER3++;
// use assembly since variable need to be   big endian
// to reduce time on irq just increment ICOUNTER
// and then on main program  add ICOUNTER IOSensorData[3].DWORD++;

#asm
        
        banksel(_ICOUNTER)
        incf (_ICOUNTER+3)^384,f
        btfss _IOCounterReset,6
        goto  INCCOUNTERIO3
        bcf _IOCounterReset,6
        movf (_COUNTER+6)^384,w
        movwf (_IOSensorData+4+ARRAY3)^384
        movf (_COUNTER+7)^384,w
        movwf (_IOSensorData+5+ARRAY3)^384
        clrf (_COUNTER+6)^384
        clrf (_COUNTER+7)^384
INCCOUNTERIO3:
        incf ((_COUNTER+7)^384),f
        skipnz
        incf ((_COUNTER+6)^384),f
        movlb 0
#endasm

        }
    }

      if(_temp&0x80)
    {
        //IO4
        IOCBFbits.IOCBF7=0;

        if(IOCounterFlag.IO4)
        {
//           IOSensorData[4].DWORD++;
//            COUNTER4++;
          // use assembly since variable need to be   big endian
// to reduce time on irq just increment ICOUNTER
// and then on main program  add ICOUNTER IOSensorData[4].DWORD++;

#asm
        banksel(_ICOUNTER)
        incf (_ICOUNTER+4)^384,f
        btfss _IOCounterReset,7
        goto  INCCOUNTERIO4
        bcf _IOCounterReset,7
        movf (_COUNTER+8)^384,w
        movwf (_IOSensorData+4+ARRAY4)^384
        movf (_COUNTER+9)^384,w
        movwf (_IOSensorData+5+ARRAY4)^384
        clrf (_COUNTER+8)^384
        clrf (_COUNTER+9)^384
INCCOUNTERIO4:
        incf ((_COUNTER+9)^384),f
        skipnz
        incf ((_COUNTER+8)^384),f
        movlb 0
#endasm

        }
    }
 }

asm ("SKIPIOC:");   // fast lane to skip IOC

// this is the second test to remove glitch
// timer 1 use by R/C Servo

    if(TMR1IE)
    if(TMR1IF)
{
    TMR1IF=0;
    if(ServoIndex<5)
    {
      //PORTB &= ServoMask
#asm
        movf _ServoMask,w
        andwf _PORTB,f
#endasm
    }
    else
    {
      // PORTA &= ServoMask
#asm
        movf _ServoMask,w
        andwf _PORTA,f
#endasm
    }
    TMR1ON=0;
  }



// Timer 2 clock system
if(TMR2IF){
 TMR2IF=0;
 if(TimerSecFlag)
 {
     ResetCounterFlag=1;
     //Tell system to Transfer/reset counter
     IOCounterReset.Byte = IOCounterFlag.Byte;

/*
     if(IOCounterFlag.IO0)
     {

#asm
        banksel(_IOSensorData)
        movf (_COUNTER+0)^384,w
        movwf (_IOSensorData+4+ARRAY0)^384
        movf (_COUNTER+1)^384,w
        movwf (_IOSensorData+5+ARRAY0+1)^384
        clrf (_COUNTER+0)^384
        clrf (_COUNTER+1)^384

#endasm

    }

     if(IOCounterFlag.IO1)
     {

#asm
        banksel(_IOSensorData)
        movf (_COUNTER+2)^384,w
        movwf (_IOSensorData+4+ARRAY1)^384
        movf (_COUNTER+3)^384,w
        movwf (_IOSensorData+5+ARRAY1)^384
        clrf (_COUNTER+2)^384
        clrf (_COUNTER+3)^384

#endasm
     }

    if(IOCounterFlag.IO2)
     {

#asm
        banksel(_IOSensorData)
        movf (_COUNTER+4)^384,w
        movwf (_IOSensorData+4+ARRAY2)^384
        movf (_COUNTER+5)^384,w
        movwf (_IOSensorData+5+ARRAY2)^384
        clrf (_COUNTER+4)^384
        clrf (_COUNTER+5)^384

#endasm
     }

    if(IOCounterFlag.IO3)
     {

#asm
        banksel(_IOSensorData)
        movf (_COUNTER+6)^384,w
        movwf (_IOSensorData+4+ARRAY3)^384
        movf (_COUNTER+7)^384,w
        movwf (_IOSensorData+5+ARRAY3)^384
        clrf (_COUNTER+6)^384
        clrf (_COUNTER+7)^384

#endasm
     }

    if(IOCounterFlag.IO4)
     {

#asm
        banksel(_IOSensorData)
        movf (_COUNTER+8)^384,w
        movwf (_IOSensorData+4+ARRAY4)^384
        movf (_COUNTER+9)^384,w
        movwf (_IOSensorData+5+ARRAY4)^384
        clrf (_COUNTER+8)^384
        clrf (_COUNTER+9)^384

#endasm
     }
*/

#asm
     // need to set bank since we play with it in assembly
     // TimerSecFlag is a bit so need to dive by 8
     // banksel should be 0 anyway but just in case
     banksel(_TimerSecFlag/8)
#endasm
  TimerSecFlag=0;
 }

 Timerms++;
 PrimaryTimerms--;
 if(PrimaryTimerms==0)
  {
     PrimaryTimerms=TIMER_100MS;
    if(WaitForEndDeciSecond)
    {
//        _TMR0=TMR0;
#asm
       movf _TMR0,w
       movwf __TMR0
#endasm

        CPSON=0;
        TMR0IE=0;
        WaitForEndDeciSecond=0;
        GotCapSenseFlag=1;
       
    }
    else if(WaitForStartDeciSecond)
    {
        WaitForStartDeciSecond=0;        
        TMR0=0;
        TMR0IF=0;
        TMR0IE=1;
        WaitForEndDeciSecond=1;
       
    }
     TimerDeciSec--;
     if(TimerDeciSec==0)
     {
//         TimerDeciSec=10;
#asm
         movlw 10
         movwf _TimerDeciSec
#endasm
         TimerSecFlag=1;
     }


  }
}

    // check serial transmit Interrupt
if(TXIE)
 if(TXIF)
  {
     // do we have a new char to send
    if(InFiFo != OutFiFo)
      {
        TXREG= SerialBuffer[OutFiFo];
//        OutFiFo++;
#asm
       movlb 0 ; select bank0
       incf _OutFiFo
#endasm
       if(OutFiFo >= SERIAL_BUFFER_SIZE)
         OutFiFo=0;
      }
     else
   if(OutFiFo == InFiFo)
     {
       // nothing in buffer  disable tx interrupt
       TXIE=0;
     }
  }



  if(TMR0IE)
  if(TMR0IF)
   {
      TMR0IF=0;
      if(Timer0Overflow)
      {
//          _TMR0_MSB++;
#asm
          banksel(__TMR0_MSB)
          incf __TMR0_MSB,f
          skipz
          incf __TMR0_MSB+1,f
#endasm
      }
      else
      {
     // got timer0 time out
       TMR0IE=0;
//        CurrentIOCycle=IO_CYCLE_DHT_ANALYZE;
#asm
       movlw IO_CYCLE_DHT_ANALYZE
       banksel(_CurrentIOCycle)
       movwf _CurrentIOCycle        
#endasm
      }
  }
    // check serial  receive
if(RCIE)
 if(RCIF)
   {
//     RcvSerialBuffer[RcvInFiFo++]= RCREG;
#asm
     banksel(_RcvInFiFo)
     movf _RcvInFiFo,w
     incf _RcvInFiFo,f
     addlw  low (_RcvSerialBuffer)
     movwf 6
     movlw high _RcvSerialBuffer
     movwf 7
     movlb 3  ; select bank 3
     movf 0x19 ,w
     movwf 1
     banksel(_RcvInFiFo)
#endasm
     if(RcvInFiFo == SERIAL_BUFFER_SIZE)
        RcvInFiFo=0;
   }
  
}

void putch(char char_out)
{
   unsigned char temp;

    SerialSum+= (unsigned char) char_out;
// increment circular pointer InFiFo
   temp = InFiFo + 1;
   if(temp >= SERIAL_BUFFER_SIZE)
     temp = 0;

//  wait  if buffer full
  while(temp == OutFiFo);

// ok write the buffer
  SerialBuffer[InFiFo]=char_out;
// now tell the interrupt routine we have a new char
InFiFo= temp;

// and enable interrupt
 TXIE=1;
}

// summing the 32 bit counter into irq will reduce irq timing
// instead we increment  8 bot  ICOUNTER and we will
// sum ICOUNTER into the 32 bits and reset ICOUNTER
// P.S. we need to disable interrupts, load ICOUNTER, reset ICOUNTER,
// enable interrupt and add it into 32 bits IOSEnsorData.dword
//
// P.S.  MODBUS IS BIG ENDIAN


void  DoSumAllCounter()
{
    di();
     if(IOCounterReset.IO0)
     {
#asm
        bcf _IOCounterReset,3
        banksel(_IOSensorData)
        movf (_COUNTER+0)^384,w
        movwf (_IOSensorData+4+ARRAY0)^384
        movf (_COUNTER+1)^384,w
        movwf (_IOSensorData+5+ARRAY0+1)^384
        clrf (_COUNTER+0)^384
        clrf (_COUNTER+1)^384

#endasm

    }
    ei();
    di();
     if(IOCounterReset.IO1)
     {

#asm
        bcf _IOCounterReset,4
        banksel(_IOSensorData)
        movf (_COUNTER+2)^384,w
        movwf (_IOSensorData+4+ARRAY1)^384
        movf (_COUNTER+3)^384,w
        movwf (_IOSensorData+5+ARRAY1)^384
        clrf (_COUNTER+2)^384
        clrf (_COUNTER+3)^384

#endasm
     }
    ei();
    di();
    if(IOCounterReset.IO2)
     {

#asm
        bcf _IOCounterReset,5
        banksel(_IOSensorData)
        movf (_COUNTER+4)^384,w
        movwf (_IOSensorData+4+ARRAY2)^384
        movf (_COUNTER+5)^384,w
        movwf (_IOSensorData+5+ARRAY2)^384
        clrf (_COUNTER+4)^384
        clrf (_COUNTER+5)^384

#endasm
     }
    ei();
    di();

    if(IOCounterReset.IO3)
     {

#asm
        bcf _IOCounterReset,6
        banksel(_IOSensorData)
        movf (_COUNTER+6)^384,w
        movwf (_IOSensorData+4+ARRAY3)^384
        movf (_COUNTER+7)^384,w
        movwf (_IOSensorData+5+ARRAY3)^384
        clrf (_COUNTER+6)^384
        clrf (_COUNTER+7)^384

#endasm
     }
    di();
    ei();

    if(IOCounterReset.IO4)
     {

#asm
        bcf _IOCounterReset,7
        banksel(_IOSensorData)
        movf (_COUNTER+8)^384,w
        movwf (_IOSensorData+4+ARRAY4)^384
        movf (_COUNTER+9)^384,w
        movwf (_IOSensorData+5+ARRAY4)^384
        clrf (_COUNTER+8)^384
        clrf (_COUNTER+9)^384

#endasm
     }
    ei();



#asm
    banksel(_IOSensorData)
    // disable interrupt
    bcf 11,7
    // load ICOUNTER
    movf (_ICOUNTER)^384,w
    // reset ICOUNTER
    clrf (_ICOUNTER)^384
    // enable interrupt
    bsf 11,7
    // finally add ICOUNTER into ioSensorData BIG ENDIAN mode
    addwf ((_IOSensorData+ARRAY0+3)^384),f
    movlw 0
    addwfc ((_IOSensorData+ARRAY0+2)^384),f
    addwfc ((_IOSensorData+ARRAY0+1)^384),f
    addwfc ((_IOSensorData+ARRAY0)^384),f

    //  IO1  COUNTER

    bcf 11,7
    movf (_ICOUNTER+1)^384,w
    clrf (_ICOUNTER+1)^384
    bsf 11,7
    addwf ((_IOSensorData+ARRAY1+3)^384),f
    movlw 0
    addwfc ((_IOSensorData+ARRAY1+2)^384),f
    addwfc ((_IOSensorData+ARRAY1+1)^384),f
    addwfc ((_IOSensorData+ARRAY1)^384),f

    //  IO2  COUNTER

    bcf 11,7
    movf (_ICOUNTER+2)^384,w
    clrf (_ICOUNTER+2)^384
    bsf 11,7
    addwf ((_IOSensorData+ARRAY2+3)^384),f
    movlw 0
    addwfc ((_IOSensorData+ARRAY2+2)^384),f
    addwfc ((_IOSensorData+ARRAY2+1)^384),f
    addwfc ((_IOSensorData+ARRAY2)^384),f

    //  IO3  COUNTER

    bcf 11,7
    movf (_ICOUNTER+3)^384,w
    clrf (_ICOUNTER+3)^384
    bsf 11,7
    addwf ((_IOSensorData+ARRAY3+3)^384),f
    movlw 0
    addwfc ((_IOSensorData+ARRAY3+2)^384),f
    addwfc ((_IOSensorData+ARRAY3+1)^384),f
    addwfc ((_IOSensorData+ARRAY3)^384),f

    //  IO4  COUNTER

    bcf 11,7
    movf (_ICOUNTER+4)^384,w
    clrf (_ICOUNTER+4)^384
    bsf 11,7
    addwf ((_IOSensorData+ARRAY4+3)^384),f
    movlw 0
    addwfc ((_IOSensorData+ARRAY4+2)^384),f
    addwfc ((_IOSensorData+ARRAY4+1)^384),f
    addwfc ((_IOSensorData+ARRAY4)^384),f
    banksel(0)
#endasm
}



unsigned char  RcvGetBufferLength(void)
{
   unsigned char temp;

   temp =  SERIAL_BUFFER_SIZE;
   temp += RcvInFiFo;
   temp -= RcvOutFiFo;

   if(temp >= SERIAL_BUFFER_SIZE)
       temp-= SERIAL_BUFFER_SIZE;
   return temp;

}

void RcvClear(void)
{
    GIE=0;
    RcvInFiFo=0;
    RcvOutFiFo=0;
    GIE=1;
}
unsigned char RcvIsDataIn(void)
 {
     return (RcvInFiFo == RcvOutFiFo ? 0 : 1);
 }

char RcvGetChar(void)
 {
    char temp;

   // wait until we received something
   while(!RcvIsDataIn());

  // get the character
    temp =  RcvSerialBuffer[RcvOutFiFo];
    RcvOutFiFo++;
    if(RcvOutFiFo >= SERIAL_BUFFER_SIZE)
      RcvOutFiFo=0;

    return temp;
}


void TXM_WAIT(void)
{
    while(TXIE);
    __delay_us(200);
    TXM_ENABLE=0;
}



void SendModbusPacket(int BufferSize)
{
    unsigned short CRC;
    unsigned char loop;
    CRC = CRC16(ModbusPacketBuffer,BufferSize);
    //RS-485 on TRANSMISSION
    ModbusOnTransmit=1;
    TXM_ENABLE=1;
    // send data
    for(loop=0;loop<BufferSize;loop++)
        putch(ModbusPacketBuffer[loop]);
    // send CRC
    putch(CRC & 0xFF);
    putch(CRC >> 8);


}

void InitModbusPacket(void)
{
    ModbusPacketBuffer[0]=Setting.SlaveAddress;
    ModbusPacketBuffer[1]=ModbusFunction;
}


void  SendFrameError(unsigned char ErrorCode)
{
    InitModbusPacket();
    ModbusPacketBuffer[1]= ModbusFunction | 0x80;
    ModbusPacketBuffer[2]= ErrorCode;
    SendModbusPacket(3);
}




void SendReadByteFrame(unsigned  char value)
{

   InitModbusPacket();
   ModbusPacketBuffer[2]= 1;  // byte count
   ModbusPacketBuffer[3]= value;
   SendModbusPacket(4);
 }

void SendReadFrame(unsigned  short value)
{
   InitModbusPacket();
   ModbusPacketBuffer[2]= 2;  // byte count
   ModbusPacketBuffer[3]= value >> 8;
   ModbusPacketBuffer[4]= value & 0xff;
   SendModbusPacket(5);
}




void SendBytesFrame(unsigned char _Address)
{
  unsigned char loop=0;
  unsigned char NByte=0;
 
  unsigned char _temp;

  _temp = Setting.IOConfig[_Address];

  if(_temp >IOCONFIG_PWM)
  {
      if(_temp<IOCONFIG_DHT11)
          NByte= 4;
      else if(_temp== IOCONFIG_SERVO)
          NByte= 0;
      else
          NByte=6;
  }
  

  if(NByte==0)
         SendFrameError(ILLEGAL_DATA_ADDRESS);
    else
     {
       InitModbusPacket();

       ModbusPacketBuffer[2]= NByte;
//  counter use interrupt
//  we need to transfer data with interrupt disable
// use FSR0 = IOSensor[_Address]
// and FSR1 = _modbusPacketBuffer[3]
// IOSensorData =0x1A0
//
//       di()
//              for(loop=0;loop<NByte;loop++)
//        {
//          _temp= tSensor.BYTE[loop];
//          ModbusPacketBuffer[3+loop]=_temp;
//        }
//       ei()

       _temp = _Address * sizeof(SensorDataUnion);
       #asm
       movlw _IOSensorData/256
       movwf FSR0H
       movlw _IOSensorData&0xff
       addwf SendBytesFrame@_temp,w
       movwf FSR0L
       movlw _ModbusPacketBuffer/256
       movwf FSR1H
       movlw (_ModbusPacketBuffer&0xff)+3
       movwf FSR1L
       movf SendBytesFrame@NByte,w
       movwf SendBytesFrame@loop
#endasm
      // ok move NBytes without interrupt
       di();
#asm
BYTELOOP:
        movf  INDF0,w
        movwi FSR1++
        incf FSR0L,f
        decfsz SendBytesFrame@loop,f
        goto BYTELOOP
#endasm
         ei();

       SendModbusPacket(NByte+3);
     }
}


void SendPresetFrame()
{
   unsigned char buffer[6];

   InitModbusPacket();

   ModbusPacketBuffer[2]= ModbusAddress >> 8;
   ModbusPacketBuffer[3]= ModbusAddress & 0xff;
   ModbusPacketBuffer[4]= ModbusData >> 8;
   ModbusPacketBuffer[5]= ModbusData & 0xff;
   SendModbusPacket(6);
   
}



unsigned char DecodeSerial(char * msg)
{
    int loop;
    unsigned char rcode;
    unsigned short CalcCRC;

    unsigned char * pt=msg;

    ModbusSlave= *(pt++);
    ModbusFunction= *(pt++);


    #define ToUSHORT    ((((unsigned short)*pt) << 8 ) | ((unsigned short) pt[1]));pt+=2;

    ModbusAddress= ToUSHORT;
    ModbusData= ToUSHORT;

    // MODBUS CRC have LSB FIRST
    ModbusCRC= *pt++;
    ModbusCRC|= ((unsigned short)(*pt)) << 8;


    CalcCRC = CRC16(ModbusBuffer,6);

   if(CalcCRC != ModbusCRC) rcode=0;
   else if(ModbusSlave==Setting.SlaveAddress) rcode=1;
   else rcode=2;

   return rcode;

}




  // Function 3  Read Holding Register
  //
  // Address 0..9: Read R/C servo on IOx if enable
  // Address 160: SlaveAddress (Node ID)
  // Address 0x10n: Read IOn Config
  // Address 250: Version Number
  // Address 251: Software ID NUMBER

void   ReadHoldingRegister()
{
    unsigned short temp;
    char Flag= 0;
    if((ModbusAddress >= 0x100) && (ModbusAddress <= 0x10a))
      temp = Setting.IOConfig[ModbusAddress - 0x100];
   else if(ModbusAddress == 160)
      temp = Setting.SlaveAddress;
   else if(ModbusAddress == 250)
      temp = RELEASE_VERSION;
   else if(ModbusAddress == 251)
      temp = SOFTWARE_ID;
   else if(ModbusAddress <10)
      temp= ServoTimer[ModbusAddress];
   else
      Flag= 1;

    if(Flag)
     SendFrameError( ILLEGAL_DATA_ADDRESS);
    else
     SendReadFrame(temp);
}


unsigned short ReadIO(unsigned char Pin)
{
  
  BadIO=0;  // clean Bad IO 
  unsigned char ioconfig = Setting.IOConfig[Pin];
  unsigned short temp;
  unsigned char mask;
  unsigned char _tempb;
  // ANALOG MODE
  if(ioconfig <= IOCONFIG_ANALOG4V)
    {
      SetAnalogConfig(Pin);  // set the analog VRef
      return ReadA2D(CSMASK[Pin]);
    } 

  // INPUT  & OUTPUT  MODE
  if(ioconfig <= IOCONFIG_OUTPUT)
  {
      return ReadIOPin(Pin);
  }
  else
      return(0xffff);
  
}

unsigned short ReadVRef()
{
  // A/D INPUT = FVR (2.048V)
  FVRCONbits.ADFVR=2;
  // A/D VREF = VDD
  ADCON1bits.ADPREF=0;
  return ReadA2D(31);
}

unsigned short ReadTSensor()
{
  // A/D VREF = VDD
  FVRCONbits.TSEN=1;
  FVRCONbits.TSRNG=0;
  ADCON1bits.ADPREF=0;
  return ReadA2D(29);
}


unsigned char  MultipleRegister(unsigned char _Address)
{
  if(Setting.IOConfig[_Address] & (IOCONFIG_CAP_SENSE_OFF | IOCONFIG_DHT11 | IOCONFIG_DS18B20 | IOCONFIG_COUNTER))
            return 1;
        return 0;
}


  // Function 4  Read Current Register
  //
  // Address 0x1000:  Current VRef 2.048V A/D value
  // Address 0x1001:  Current Build-in Temperature Sensor
  // Address 0xn0:  Read current IOn
void   ReadCurrentRegister()
{
    unsigned short temp;
    unsigned char IOn;

  
       temp=0;
       BadIO=0;
       if(ModbusAddress<0xA0)
       {
           IOn = ModbusAddress >>4;
       
       if(MultipleRegister(IOn))
       {
           SendBytesFrame(IOn);
           return;
       }
       else
           temp= ReadIO(IOn);
       }
       else if(ModbusAddress==0x1000)
           temp = ReadVRef(); // Read 2.048V reference Value
       else if(ModbusAddress==0x1001)
           temp = ReadTSensor(); // Read Build-in Temperature sensor
       else
           BadIO=1;
        
       if(BadIO)
         SendFrameError(ILLEGAL_DATA_ADDRESS);
       else
         SendReadFrame(temp);
   
}


void ReadInputStatus()
{
  // Read Coil ou Read Input Status
    unsigned char _tmp;

    
    if(ModbusAddress < INPUT_COUNT)
    {
     if(ModbusAddress < 5)
         _tmp = PORTB;
     else
         _tmp = PORTA;
        _tmp&=IOMASK[ModbusAddress];
        
      SendReadByteFrame(_tmp);

    }
   else
      SendFrameError( ILLEGAL_DATA_ADDRESS);  
}

void ForceSingleCoil()
{
    unsigned char mask;
    if(ModbusAddress < INPUT_COUNT)
    {
       if(Setting.IOConfig[ModbusAddress] == IOCONFIG_OUTPUT)
         {
           mask = IOMASK[ModbusAddress];
           if(ModbusAddress<5)
           {
               if(ModbusData==0)
                   PORTB &= ~mask;
               else
                   PORTB |= mask;

           }
           else
           {
               if(ModbusData==0)
                   PORTA &= ~mask;
               else
                   PORTA |= mask;
           }

           SendPresetFrame();
           return;     
         }
    }
    SendFrameError( ILLEGAL_DATA_ADDRESS);
}


  // Function 6 Preset Single Register
  //
  // Address 0x10n: IOConfig I0 0..9
  // Address 0x0: Set RCServo0 and clear counter accumulator
  // Address 0x1: Set RCServo1 and clear counter accumulator
  // Address 160: Slave Address
  // Address 0x1ff:  if value is 0x5678 Enable Change in  configuration
  // Address 0xAA55: if value is 0x1234 this mean reset

void PresetSingleRegister()
{
  unsigned char oldConfig;
  unsigned char temp;
  if(ModbusAddress == 0x1ff)
  {
      EnableConfigChange= (ModbusData == 0x5678);
  }
  else if(ModbusAddress == 0xAA55)
  {

      if(ModbusData == 0x1234)
      {
        ForceReset=1;
        WDTCON = 0b00010001;
        SendPresetFrame();
        // put watch dog 256ms
     
      }
      else
         SendFrameError(ILLEGAL_DATA_ADDRESS);
  }
  else if((ModbusAddress >=0x100) && (ModbusAddress <= 0x109))
    {
      if(EnableConfigChange)
      {
      temp = ModbusAddress - 0x100;
      BadIO=0;
      oldConfig=Setting.IOConfig[temp];
      Setting.IOConfig[temp]=ModbusData;
      SetIOConfig(temp);
      if(BadIO)
        {
         Setting.IOConfig[temp]=oldConfig;
         SetIOConfig(temp);
         SendFrameError(ILLEGAL_DATA_ADDRESS);
        }
      else
        {
          SendPresetFrame();
          SaveSetting();
        }
      EnableConfigChange=1;
      }
      else
          SendPresetFrame();
    }
    else if(ModbusAddress <10)
    {
        if(!SetPWMConfig(ModbusAddress,ModbusData)) // This only works if available
        {
           if(Setting.IOConfig[ModbusAddress]==IOCONFIG_COUNTER)
        {
            if(ModbusData==0)
                IOSensorData[ModbusAddress].DWORD=0;
        }
        else
         ServoTimer[ModbusAddress]=ModbusData;
        }

        SendPresetFrame();
    }
   else if(ModbusAddress == 160)
    {
       if(EnableConfigChange)
       {
      Setting.SlaveAddress=ModbusData;
      SaveSetting();
       }
      SendPresetFrame();
    }
    else
       SendFrameError( ILLEGAL_DATA_ADDRESS);
}


void ExecuteCommand(void)
{

  if(ModbusSlave != Setting.SlaveAddress)
     return;    // this is not our Slave Address! just forget it


      __delay_us(100);

 // if(ModbusLRC != ModbusCheckSum)
 //     return; // invalide check sum we should deal with it

  if(ModbusFunction == 1)
      ReadInputStatus();
  else if(ModbusFunction == 2)
      ReadInputStatus();
  else if(ModbusFunction == 3)
      ReadHoldingRegister();
  else if(ModbusFunction == 4)
      ReadCurrentRegister();
  else if(ModbusFunction == 5)
      ForceSingleCoil();  
  else if(ModbusFunction == 6)
      PresetSingleRegister();
  else
     SendFrameError(ILLEGAL_FUNCTION);
}



 main(void){
     unsigned char loop;
     unsigned char rcode;


#ifndef USE_EXTERNAL_XTAL

 // assume 32Mhz
 OSCCON		= 0b11110000;	// 32MHz  internal clock

#endif


 // assume 32Mhz
 OPTION_REG	= 0b00000010;	// pullups on, TMR0 @ Fosc/4/8
 

 ANSELA		= 0;	// NO Analog
 ANSELB         =0;
 PORTA   	= 0b00100000;
 WPUA		= 0b00111111;	// pull-up ON

 


 INTCON		= 0b00000000;	// no interrupt


 // Set Default Watch dog time to 16 sec
 WDTCON = 0b00011101; // bit 0 is ignore
 ForceReset=0;
#asm   
 CLRWDT
#endasm


 // Load Modbus and IO configuration

 LoadSetting();


 TRISA		= 0b00101011;	// RA0,RA1,RA3,RA5 INPUT , RA2,RA4 OUTPUT
 TXM_ENABLE=0;

 // set serial com with 57600 baud
//alternate pin
 APFCON0 = 0b00000000;
 APFCON1 = 0b00000000;
 
    
 TXSTA = 0b10000010;
 RCSTA = 0;


#if BAUD == 9600
 BRGH =0; //8mhz =>1;
 BRG16 = 1;
 SYNC =0;
 SPBRGL = 207; // assume 32Mhz clock
 SPBRGH =0;
#elif BAUD == 19200
  BRGH =0;
 BRG16 = 1;
 SYNC =0;

 SPBRGL = 103;
 SPBRGH = 0;

#elif BAUD == 38400
 BRGH = 1;
 BRG16=1;
 SYNC =0;
 SPBRG = 208;
#elif BAUD == 115200
 // assume  baud 115200
 BRGH =1;
 BRG16 = 1;
 SYNC =0;


 SPBRGL = 68; // assume 32Mhz clock
 SPBRGH =0;

#else

// assume  baud 57600
 BRGH =1;
 BRG16 = 1;
 SYNC =0;


 SPBRGL = 138; // assume 32Mhz clock
 SPBRGH =0;

#endif


 TXEN =1;   // enable transmitter
 SPEN = 1;  // enable serial port
 CREN = 1;  // enable receiver
 TXIE =0;   // disable transmit interrupt
 RCIF =0;   // clear received flag
 TXIF = 0;
 SCKP = 0;
 ABDEN = 0;
// reset interrupt fifo buffer
 InFiFo=0;
 OutFiFo=0;
 RcvInFiFo=0;
 RcvOutFiFo=0;


 GIE = 1;
 PEIE =1;   // enable peripheral
 RCIE =1;   // Enable received interrupt
 IOCBP =0;
 IOCBN = 0;
 IOCBF = 0;
 IOCIE = 1; // enable interrupt on change


IOCounterFlag.Byte=DHTFlag=0;

ModbusOnTransmit=0;

 Init1msTimer() ;
 InitA2D() ;

 // prepare IO pin
 for(loop=0;loop<INPUT_COUNT;loop++)
 {
    EnableConfigChange=1; // enable IOCONFIG CHANGE
 SetIOConfig(loop);
 }
 EnableConfigChange=0;

 // timer 4 use for pwm
 T4CON = 0b00000111;  // 1:16 timer4 ON
 CCPTMRS= 0b01010101; // all pwm to timer4
 PR4=0xff;
 TMR4=0;
 TMR4IE=0;
 TMR4IF=0;
 
    // clear Modbus system first
 RcvClear();
 ModbusFramePointer=0;
 // five second delay for IO stabilisation
  __delay_ms(5000);
 // cputs("IO Multi 10 V1.0\n\r");

  ResetIOCycle();

  for(loop=0;loop<INPUT_COUNT;loop++)
      ServoTimer[loop]=0;

// timer 1
   ServoIndex=INPUT_COUNT;
    // assume 32Mhz clock
   T1CON = 0b00110000;  // fsoc/4/8  (1Mhz timer clock
   TMR1GE=0;
   TMR1IF=0;
   TMR1IE=0;
   TMR1ON=0;





 while(1)
 {
     // clear watch dog
     if(!ForceReset)
     {
     #asm
      CLRWDT
     #endasm
     }
              // No more transmission. put the system back on reception
       // let's
       if(!ModbusOnTransmit)
           TXM_ENABLE=0;
     if(ModbusOnTransmit)
     {
         if(!TXIE)
         {
             __delay_us(200);
             TXM_ENABLE=0;
             ModbusOnTransmit=0;
         }
     }
     else
     if(RcvIsDataIn())
     {

         ModbusBuffer[ModbusFramePointer++]=RcvGetChar();
         if(ModbusFramePointer>=8)
         {
             ModbusFramePointer=8;
             rcode = DecodeSerial(ModbusBuffer);
          if(rcode==1)
          {
              ExecuteCommand();
             ModbusFramePointer=0;
          }
          else if(rcode ==2)
          {
              // ok not this slave
              ModbusFramePointer=0;
          }
          else
          {
              // something wrong then just shift data
              for(loop=1;loop<8;loop++)
                  ModbusBuffer[loop-1]=ModbusBuffer[loop];
              ModbusFramePointer--;
          }
         }
     }

     DoIOCycle();
     DoRCServo();
     DoSumAllCounter();

 }

}





