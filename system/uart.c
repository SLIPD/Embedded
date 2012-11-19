#include "uart.h"

#include "efm32_cmu.h"
#include "efm32_int.h"

#include "system.h"

#include <stdbool.h>
#include <stdlib.h>

#include "scheduler.h"

/* variables */
typedef struct
{
	
	uint8_t *data;
	uint16_t position,
		size;
	bool *complete;

} uart_transfer_t;

uart_transfer_t *tx_transfer = NULL,
	*rx_transfer = NULL;

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
	}
	
	UART1->CLKDIV = 256 * ((CMU_ClockFreqGet(cmuClock_HF)) / (9600 * 16) - 1);

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
	
	if (tx_transfer == NULL)
	{
		USART_IntDisable(UART1, UART_IF_TXBL);
		return;
	}
	
	if (tx_transfer->position == tx_transfer->size)
	{
		*(tx_transfer->complete) = true;
		tx_transfer = NULL;
		USART_IntDisable(UART1, UART_IF_TXBL);
		return;
	}
	
	UART1->TXDATA = tx_transfer->data[tx_transfer->position];
	tx_transfer->position++;
	
}

void UART1_RX_IRQHandler()
{
	
	if (!(UART1->STATUS & UART_STATUS_RXDATAV))
		return;
	
	if (rx_transfer == NULL)
	{
		USART_IntDisable(UART1, UART_IF_RXDATAV);
		return;
	}
	
	rx_transfer->data[rx_transfer->position] = UART1->RXDATA;
	rx_transfer->position++;
	
	if (rx_transfer->position == rx_transfer->size)
	{
		*(rx_transfer->complete) = true;
		rx_transfer = NULL;
		USART_IntDisable(UART1, UART_IF_RXDATAV);
		return;
	}
	
}

void UART1_Send(uint8_t *data, uint16_t size)
{
	
	bool complete = false;
	
	uart_transfer_t transfer;
	transfer.data = data;
	transfer.position = 0;
	transfer.size = size;
	transfer.complete = &complete;
	
	do
	{
		
		INT_Disable();
		if (tx_transfer == NULL)
		{
			tx_transfer = &transfer;
			INT_Enable();
			break;
		}
		INT_Enable();
		SCHEDULER_Wait(UART1_SEND_FLAG);
		
	}
	while(1);
	
	USART_IntEnable(UART1, USART_IF_TXBL);
	
	while(!complete);
	
	SCHEDULER_Release(UART1_SEND_FLAG);
	
}

void UART1_Recv(uint8_t *data, uint16_t size)
{
	
	bool complete = false;
	
	uart_transfer_t transfer;
	transfer.data = data;
	transfer.position = 0;
	transfer.size = size;
	transfer.complete = &complete;
	
	do
	{
		
		INT_Disable();
		if (rx_transfer == NULL)
		{
			rx_transfer = &transfer;
			INT_Enable();
			break;
		}
		INT_Enable();
		SCHEDULER_Wait(UART1_RECV_FLAG);
		
	}
	while(1);
	
	USART_IntEnable(UART1, USART_IF_RXDATAV);
	
	while(!complete);
	
	SCHEDULER_Release(UART1_RECV_FLAG);
	
}
