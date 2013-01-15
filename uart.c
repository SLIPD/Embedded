#include "uart.h"

#include "efm32_cmu.h"
#include "efm32_int.h"
#include "efm32_usart.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* variables */
uint8_t uartMemory[UART_BUFFER_SIZE];
uint16_t uartWritePos = 0,
	uartLen = 0,
	uartReadPos = 0;

void (*uart_rx_handler)(uint8_t) = NULL;

/* prototyes */


/* functions */
void UART1_Init(uint8_t location)
{
	
	uint32_t uart_location;
	switch (location)
	{
	case 0:
		uart_location = UART_ROUTE_LOCATION_LOC0;
		break;
	case 1:
		uart_location = UART_ROUTE_LOCATION_LOC1;
		break;
	case 2:
		uart_location = UART_ROUTE_LOCATION_LOC2;
		break;
	case 3:
		uart_location = UART_ROUTE_LOCATION_LOC3;
		break;
	}
	
	UART1->CLKDIV = 256 * ((CMU_ClockFreqGet(cmuClock_HF)) / (115200 * 16) - 1);

	UART1->CMD = UART_CMD_TXEN | UART_CMD_RXEN;

	GPIO->P[4].DOUT |= (1 << 2);
	GPIO->P[4].MODEL =
						GPIO_P_MODEL_MODE2_PUSHPULL
					| GPIO_P_MODEL_MODE3_INPUT;

	UART1->ROUTE = uart_location
          | UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;
    
}

void UART1_TX_IRQHandler()
{
	
	if (!(UART1->STATUS & UART_STATUS_TXBL))
		return;
	
	if (uartLen == 0)
	{
		USART_IntDisable(UART1, UART_IF_TXBL);
		return;
	}
	
	UART1->TXDATA = uartMemory[uartReadPos];
	uartReadPos = (uartReadPos + 1) % UART_BUFFER_SIZE;
	uartLen--;
	
}

void UART1_RX_IRQHandler()
{
	
	if (!(UART1->STATUS & UART_STATUS_RXDATAV))
		return;
	
	if (uart_rx_handler == NULL)
	{
		USART_IntDisable(UART1, UART_IF_RXDATAV);
		return;
	}
	
	uart_rx_handler(UART1->RXDATA);
	
}

void UART1_Send(uint8_t *data, uint16_t size)
{
	
	uint8_t *pos = data;

	while (size > 0)
	{
        
        USART_IntEnable(UART1, UART_IF_TXBL);
		while (uartLen >= UART_BUFFER_SIZE);

		INT_Disable();
		
		if (uartReadPos <= uartWritePos)
		{
			uint16_t toWrite = (UART_BUFFER_SIZE - uartWritePos > size) ? size : UART_BUFFER_SIZE - uartWritePos;
			memcpy((void*)&uartMemory[uartWritePos], (void*)pos, toWrite);
			pos += toWrite;
			size -= toWrite;
			uartLen = (uartLen + toWrite) % UART_BUFFER_SIZE;
			uartWritePos = (uartWritePos + toWrite) % UART_BUFFER_SIZE;
		}
		else
		{
			uint16_t toWrite = (UART_BUFFER_SIZE - uartLen > size) ? size : UART_BUFFER_SIZE - uartLen;
			memcpy((void*)&uartMemory[uartWritePos], (void*)pos, toWrite);
			pos += toWrite;
			size -= toWrite;
			uartLen = (uartLen + toWrite) % UART_BUFFER_SIZE;
			uartWritePos = (uartWritePos + toWrite) % UART_BUFFER_SIZE;
		}
		INT_Enable();

	}
	
	USART_IntEnable(UART1, UART_IF_TXBL);
	
}

void UART1_SetRecvHandler(void (*uart_handler)(uint8_t))
{
	
	uart_rx_handler = uart_handler;
	USART_IntEnable(UART1, USART_IF_RXDATAV);
	
}

void UART1_ClearRecvHandler()
{
	
	uart_rx_handler = NULL;
	
}