#ifndef __CONFIG_H__
	
	#define __CONFIG_H__
	
	#define BASESTATION
	
	#ifdef BASESTATION
		#define NODE_ID 0
	#else
		#define NODE_ID 1
	#endif

	#define NODE_CHANNEL 102

	#include <stdarg.h>
    #include <stdio.h>
    #include <stdint.h>
    #include <stdbool.h>
    #include <string.h>

    #include "uart.h"

    static inline void TRACE_Init()
    {
        UART1_Init(3);
    }
    
    /*
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
    */
    
    static inline void TRACE(char *msg)
    {
        
        UART1_Send((uint8_t*)msg, strlen(msg));
        
    }

#endif