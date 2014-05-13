/* 
 * File:   RCServo.h
 * Author: daniel
 *
 * Created on 9 avril 2014, 17:59
 */


#ifndef RCSERVO_H
#define	RCSERVO_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "IOConfig.h"


extern void DoRCServo(void);
extern unsigned char ServoIndex;
extern unsigned short ServoTimer[INPUT_COUNT];
extern short ServoIdleTime;

/*
 * Servo Index
 *
 * IO0 Index=0
 * IO1       1
 * IO2       2
 * else  delay 20ms - Ios.delay



 */


#ifdef	__cplusplus
}
#endif

#endif	/* RCSERVO_H */

