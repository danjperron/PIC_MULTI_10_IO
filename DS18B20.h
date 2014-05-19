/* 
 * File:   DS18B20.h
 * Author: Daniel
 *
 * Created on April 4, 2014, 2:50 PM
 */

#ifndef DS18B20_H
#define	DS18B20_H

#define IO_CYCLE_DS18B20_START 9
#define IO_CYCLE_DS18B20_RESET_WAIT 10
#define IO_CYCLE_DS18B20_RESET_CHECK_FOR_DEVICE 11
#define IO_CYCLE_DS18B20_RESET_WAIT2 12
#define IO_CYCLE_DS18B20_SKIP_ROM 13
#define IO_CYCLE_DS18B20_CONVERT_T 14
#define IO_CYCLE_DS18B20_WAIT      15
#define IO_CYCLE_DS18B20_RESET2    16
#define IO_CYCLE_DS18B20_RESET2_WAIT    17
#define IO_CYCLE_DS18B20_RESET2_CHECK_FOR_DEVICE 18
#define IO_CYCLE_DS18B20_RESET2_WAIT2    19
#define IO_CYCLE_DS18B20_SKIP_ROM2 20
#define IO_CYCLE_DS18B20_READ_SCRATCHPAD 21
#define IO_CYCLE_DS18B20_READ_BYTE 22


#define DS18B20_SKIP_ROM 		0xCC
#define DS18B20_CONVERT_T 		0x44
#define DS18B20_READ_SCRATCHPAD         0xBE


extern void DoDS18B20Cycle(void);


#endif	/* DS18B20_H */

