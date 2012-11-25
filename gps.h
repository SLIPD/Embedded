/* 

 * File:   gps.h

 * Author: Robbie

 *

 * Created on 29 October 2012, 23:51

 */
#ifndef __GPS_H__

#define __GPS_H__

#include <efm32.h>

typedef struct 
{
    int32_t lon;
    int32_t lat;
    int32_t alt; //in cm

} GPS_Vector_Type __attribute__ ((packed));

// Prototypes
void leuart_send_array(LEUART_TypeDef *uart, uint8_t *msg, uint32_t len);
void GPS_Init();
void GPS_Read(GPS_Vector_Type *vector);
void GPS_GetFix();
void switchMode();
void GPS_GetPrecision(uint8_t target_precision);

#endif  /* __GPS_H__ */