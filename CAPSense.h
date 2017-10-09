/* 
 * File:   CAPSENSE.H
 * Author: Daniel
 *
 * Created on April 8, 2014, 2:42 PM
 */


#ifdef __XC__
#include <xc.h>
#else
#include <htc.h>
//#include <stddef.h>
#endif


#ifndef CAPSENSE_H
#define	CAPSENSE_H

#ifdef	__cplusplus
extern "C" {
#endif

void DoCAPSENSECycle(void);


extern bit  WaitForStartDeciSecond;
extern bit  WaitForEndDeciSecond;
extern bit  GotCapSenseFlag;


#ifdef	__cplusplus
}
#endif

#endif	/* CAPSENSE_H */

