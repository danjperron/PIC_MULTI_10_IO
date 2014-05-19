#ifndef DHT22
#define DHT22


#define IO_CYCLE_DHT_ANALYZE 50
        
extern unsigned char DHTBitBuffer[47];
extern unsigned char DHTBufferIndex;



#define DHT22_BIT_COUNT         40

       

extern void DoDHT22Cycle(void);
extern void  DHT22IOCBF(void);
extern void DHT22Error(void);
extern void CheckDHTBitBuffer(void);


#endif