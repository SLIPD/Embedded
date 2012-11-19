#ifndef __TRACE_H__
#define __TRACE_H__

#include <stdint.h>
#include <stdbool.h>

#include "uart.h"

static inline void TRACE(char *msg)
{
	UART1_Send((uint8_t*)msg, strlen(msg));
}

#endif