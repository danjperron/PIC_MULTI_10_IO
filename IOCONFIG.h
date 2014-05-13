#ifndef IOCONFIG
#define IOCONFIG

//  IOCONFIG
// ATTENTION IOCONFIG_ANALOG...  VALUE ARE DIRECTLY RELATED TO FVRCON.ADFVR
#define IOCONFIG_ANALOGVDD 0  // don't change
#define IOCONFIG_ANALOG1V  1  //   "     "
#define IOCONFIG_ANALOG2V  2  //   "     "
#define IOCONFIG_ANALOG4V  3  //   "     "
#define IOCONFIG_INPUT     4
#define IOCONFIG_INPUT_PULLUP 5
#define IOCONFIG_OUTPUT    6
#define IOCONFIG_PWM       7

#define IOCONFIG_CAP_SENSE_OFF    8
#define IOCONFIG_CAP_SENSE_LOW 	  9
#define IOCONFIG_CAP_SENSE_MEDIUM 10
#define IOCONFIG_CAP_SENSE_HIGH   11


#define IOCONFIG_DHT11     16
#define IOCONFIG_DHT22     17
#define IOCONFIG_DS18B20   32
#define IOCONFIG_SERVO     64
#define IOCONFIG_COUNTER   128


typedef union {
    struct {
        unsigned NO_USED           :3;
        unsigned CAP_SENSE         :1;
        unsigned DHT               :1;
        unsigned DS18B20           :1;
        unsigned SERVO             :1;
        unsigned COUNTER           :1;
    };
    unsigned char Config;
} ConfigUnion;





// if we use a RS485 DRIVER without the need to switch
// direction, we wil have an other input

extern const unsigned char IOMASK[11];
extern const unsigned char NOT_IOMASK[11];
extern const unsigned char CSMASK[10];
#define INPUT_COUNT 10

#define IO0 RB3
#define IO0_TRIS  TRISBbits.TRISB3
#define IO0_ANSEL ANSELBbits.ANSB3
#define IO0_AN_CHANNEL 9
#define IO0_PULLUP WPUBbits.WPUB3


#define IO1 RB4
#define IO1_TRIS  TRISBbits.TRISB4
#define IO1_ANSEL ANSELBbits.ANSB4
#define IO1_AN_CHANNEL 8
#define IO1_PULLUP WPUBbits.WPUB4

#define IO2 RB5
#define IO2_TRIS  TRISBbits.TRISB5
#define IO2_ANSEL ANSELBbits.ANSB5
#define IO2_AN_CHANNEL 7
#define IO2_PULLUP WPUBbits.WPUB6

#define IO3 RB6
#define IO3_TRIS  TRISBbits.TRISB6
#define IO3_ANSEL ANSELBbits.ANSB6
#define IO3_AN_CHANNEL 5
#define IO3_PULLUP WPUBbits.WPUB6

#define IO4 RB7
#define IO4_TRIS  TRISBbits.TRISB7
#define IO4_ANSEL ANSELBbits.ANSB7
#define IO4_AN_CHANNEL 6
#define IO4_PULLUP WPUBbits.WPUB7

#define IO5 RA0
#define IO5_TRIS  TRISAbits.TRISA0
#define IO5_ANSEL ANSELAbits.ANSA0
#define IO5_AN_CHANNEL 0


#define IO6 RA1
#define IO6_TRIS  TRISAbits.TRISA1
#define IO6_ANSEL ANSELAbits.ANSA1
#define IO6_AN_CHANNEL 1

#define IO7 RA2
#define IO7_TRIS  TRISAbits.TRISA2
#define IO7_ANSEL ANSELAbits.ANSA2
#define IO7_AN_CHANNEL 2

#define IO8 RA3
#define IO8_TRIS  TRISAbits.TRISA3
#define IO8_ANSEL ANSELAbits.ANSA3
#define IO8_AN_CHANNEL 3

#define IO9 RA4
#define IO9_TRIS  TRISAbits.TRISA4
#define IO9_ANSEL ANSELAbits.ANSA4
#define IO9_AN_CHANNEL 4



typedef struct{
unsigned char IOConfig[INPUT_COUNT];
unsigned char SlaveAddress;
}SettingStruct;


extern SettingStruct Setting;

//#define BAUD 9600
//#define BAUD 115200
#define BAUD 57600

#define USE_EXTERNAL_XTAL

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 32000000
#endif

#endif