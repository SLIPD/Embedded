#include "trace.h"

#include "string.h"
#include "efm32_gpio.h"
#include "efm32_cmu.h"
#include "efm32_usart.h"
#include "efm32_int.h"

#include "led.h"
#include "config.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "queue.h"

/* variables */
#define TRACE_BUF_SIZE 2048
static uint8_t trace_buf[TRACE_BUF_SIZE],
	trace_writePosition = 0,
	trace_readPosition = 0;

/* prototypes */

/* functions */
void TRACE_Init()
{

	UART1->CLKDIV = 256 * ((CMU_ClockFreqGet(cmuClock_HF)) / (115200 * 16) - 1);

	UART1->CMD = UART_CMD_TXEN | UART_CMD_RXEN;

	GPIO->P[4].DOUT |= (1 << 2);
	GPIO->P[4].MODEL =
						GPIO_P_MODEL_MODE2_PUSHPULL
					| GPIO_P_MODEL_MODE3_INPUT;

	UART1->ROUTE = UART_ROUTE_LOCATION_LOC3
          | UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;

}

void UART1_TX_IRQHandler()
{
	
	if (!(UART1->STATUS & UART_STATUS_TXBL))
		return;
	
	if (trace_readPosition != trace_writePosition)
	{
		UART1->TXDATA = trace_buf[trace_readPosition];
		trace_readPosition = (trace_readPosition + 1) % TRACE_BUF_SIZE;
	}
	else
	{
		USART_IntDisable(UART1, UART_IF_TXBL);
	}
	
}

void TRACE(char *format, ...)
{
	
	#ifndef BASESTATION
		
		//INT_Disable();
		
		char msg[512];
		
		va_list args;
		va_start( args, format );
		vsprintf(msg, format, args );
		va_end( args );
		
		int i;
		for (i = 0; i < strlen(msg); i++)
		{
			//trace_buf[trace_writePosition] = msg[i];
			//trace_writePosition = (trace_writePosition + 1) % TRACE_BUF_SIZE;
			while (!(UART1->STATUS & UART_STATUS_TXBL));
			UART1->TXDATA = msg[i];
		}
		
		//USART_IntEnable(UART1, UART_IF_TXBL);
		
		//INT_Enable();
		
	#endif
	
}
