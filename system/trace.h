#ifndef __TRACE_H__
#define __TRACE_H__

#include <stdint.h>
#include <stdbool.h>

#include "uart.h"

static inline void TRACE_Init()
{
	UART1_Init(3);
}

static inline void TRACE(char *format, ...)
{
	char message[255];
	int i = 0;
	for(i = 0; i < 255; i++){
		message[i] = 0;
	}
	
	
	va_list args;
    va_start( args, format );
 		vsprintf(message, format, args );
 		UART1_Send((uint8_t*)message, strlen(message));
    va_end( args );

}


#endif