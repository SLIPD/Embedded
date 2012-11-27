/* 

 * File:   gps.h

 * Author: Robbie

 *

 * Created on 29 October 2012, 23:51

 */
#ifndef __GPS_H__

#define __GPS_H__

#include <efm32.h>
#include "stdbool.h"
typedef struct 
{
    int32_t lon;
    int32_t lat;
    int16_t alt; //in cm

} GPS_Vector_Type __attribute__ ((packed));

// Prototypes
void leuart_send_array(LEUART_TypeDef *uart, uint8_t *msg, uint32_t len);
void GPS_Init();
bool GPS_Read(GPS_Vector_Type *vector);
void GPS_GetLastPosition(GPS_Vector_Type *vector);
void  GPS_Main();
void GPS_GetFix();
void switchMode();
void GPS_GetPrecision(uint8_t target_precision);
uint8_t nmea_generateChecksum(char *strPtr) ;
void leuart_send_str(LEUART_TypeDef *uart, char *msg);  
void Reset();
void GPS_WarmReset(int32_t x , int32_t y , int32_t z , uint32_t seconds , uint16_t week);
#endif  /* __GPS_H__ */