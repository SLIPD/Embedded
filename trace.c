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
static uint8_t trace_buf[TRACE_BUF_SIZE];
uint32_t trace_writePosition = 0,
	trace_readPosition = TRACE_BUF_SIZE-1;
bool trace_enabled = false;

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

	#ifndef BASESTATION
		TRACE_Enable();
	#endif

}

void UART1_TX_IRQHandler()
{
	
	if (!(UART1->STATUS & UART_STATUS_TXBL))
		return;
	
	if ((trace_readPosition+1)%TRACE_BUF_SIZE != trace_writePosition && trace_enabled)
	{
		UART1->TXDATA = trace_buf[(trace_readPosition+1)%TRACE_BUF_SIZE];
		trace_readPosition = (trace_readPosition + 1) % TRACE_BUF_SIZE;
	}
	else
	{
		USART_IntDisable(UART1, UART_IF_TXBL);
	}
	
}

void TRACE_Enable()
{
	trace_enabled = true;
}

void TRACE(char *format, ...)
{
	
	LED_Toggle(GREEN);
	
	char msg[512];
	
	va_list args;
	va_start( args, format );
	vsprintf(msg, format, args );
	va_end( args );
	
	#ifdef BASESTATION
		
		uint16_t length = strlen(msg),
			position = 0;
		uint8_t packet[32];
		packet[0] = 0xFE;
		
		int i;
		for (i = 0; i < length; i++)
		{
			
			packet[1 + (i % 31)] = msg[position++];
			if ((1 + (i % 31)) % 32 == 0)
			{
				TRACE_SendPayload(packet,32);
			}
			
		}
		while (i % 31)
		{
			packet[i+1] = 0;
			i++;
		}
		TRACE_SendPayload(packet,32);
		
	#else
		TRACE_SendPayload(msg,strlen(msg));
	#endif
	
}

void TRACE_SendPayload(uint8_t *payload, uint16_t size)
{
	
	INT_Disable();
	
	int i;
	for (i = 0; i < size; i++)
	{
		trace_buf[trace_writePosition] = payload[i];
		trace_writePosition = (trace_writePosition + 1) % TRACE_BUF_SIZE;
	}
	
	USART_IntEnable(UART1, UART_IF_TXBL);
	
	INT_Enable();
	
}
