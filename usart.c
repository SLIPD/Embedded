#include "usart.h"

#include <stdbool.h>

#include "efm32_int.h"
#include "efm32_usart.h"

#include "queue.h"

/* variables */
typedef struct
{
	
	bool active;
	uint8_t buffer[32];
	uint16_t txPosition, rxPosition, size;
	void (*cs) (USART_ChipSelect);
	void (*cb) (uint8_t *buffer, uint16_t size);
	
} usart_transfer_t;

usart_transfer_t cur_ut,
	ut_mem[16];
queue_t ut_q;

/* prototypes */
void usart_enableInterrupts(USART_TypeDef *usart);
void usart_disableInterrupts(USART_TypeDef *usart);
void USART_TX(USART_TypeDef *usart, uint8_t usart_idx);
void USART_RX(USART_TypeDef *usart, uint8_t usart_idx);

/* functions */
void USART_Init(USART_TypeDef *usart, uint8_t usart_idx, uint8_t location)
{
	
	QUEUE_Init(&ut_q, (uint8_t*)ut_mem, sizeof(usart_transfer_t), 16);
	
	uint32_t usart_location;
	switch (location)
	{
	case 0:
		usart_location = USART_ROUTE_LOCATION_LOC0;
		break;
	case 1:
		usart_location = USART_ROUTE_LOCATION_LOC1;
		break;
	case 2:
		usart_location = USART_ROUTE_LOCATION_LOC2;
		break;
	}
	
	USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;
	
	usartInit.msbf = true;
	usartInit.clockMode = usartClockMode0;
	usartInit.baudrate = 5000000;
	USART_InitSync(usart, &usartInit);
	usart->ROUTE = (usart->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | usart_location;
	usart->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;
	
	cur_ut.active = false;
	
}

void usart_enableInterrupts(USART_TypeDef *usart)
{
	USART_IntEnable(usart,USART_IF_TXBL);
	USART_IntEnable(usart,USART_IF_RXDATAV);
}

void usart_disableInterrupts(USART_TypeDef *usart)
{
	USART_IntDisable(usart,USART_IF_TXBL);
}

void USART0_TX_IRQHandler()
{
	
	USART_TX(USART0,0);
	
}

void USART0_RX_IRQHandler()
{
	
	USART_RX(USART0,0);
	
}

void USART2_TX_IRQHandler()
{
	
	USART_TX(USART2,2);
	
}

void USART2_RX_IRQHandler()
{
	
	USART_RX(USART2,2);
	
}


void USART_RX(USART_TypeDef *usart, uint8_t usart_idx)
{
	
	if (!(usart->STATUS & USART_STATUS_RXDATAV))
	{
		return;
	}
	
	usart_transfer_t *transfer = &cur_ut;
	
	if (transfer->active == false)
	{
		volatile uint8_t scratch;
		scratch = usart->RXDATA;
		usart_disableInterrupts(usart);
		return;
	}
	
	transfer->buffer[transfer->rxPosition] = usart->RXDATA;
	transfer->rxPosition++;
	
	if (transfer->rxPosition == transfer->size)
	{
		transfer->cs(HIGH);
		transfer->active = false;
		if (transfer->cb != NULL)
		{
			transfer->cb(transfer->buffer,transfer->size);
		}
		QUEUE_Read(&ut_q,(uint8_t*)transfer);
		USART_IntEnable(usart,USART_IF_TXBL);
		return;
	}
	
}

void USART_TX(USART_TypeDef *usart, uint8_t usart_idx)
{
	
	if (!(usart->STATUS & USART_STATUS_TXBL))
		return;
	
	usart_transfer_t *transfer = &cur_ut;
	
	if (transfer->active == false)
	{
		
		if (!QUEUE_Read(&ut_q,(uint8_t*)transfer))
		{
			usart_disableInterrupts(usart);
			return;
		}
		
	}
	
	if (transfer->txPosition >= transfer->size)
	{
		USART_IntDisable(usart,USART_IF_TXBL);
		return;
	}
	
	if (transfer->txPosition == 0)
	{
		transfer->cs(LOW);
		transfer->active = true;
	}
	
	usart->TXDATA = transfer->buffer[transfer->txPosition];
	transfer->txPosition++;
	
}

void USART_Transfer(USART_TypeDef *usart, uint8_t usart_idx, uint8_t *buffer, uint16_t size, void (*cs)(USART_ChipSelect), void (*cb)(uint8_t *buffer, uint16_t size))
{
	
	// create transfer
	
	usart_transfer_t transfer;
	transfer.active = false;
	memcpy(transfer.buffer,buffer,size);
	transfer.txPosition = 0;
	transfer.rxPosition = 0;
	transfer.size = size;
	transfer.cs = cs;
	transfer.cb = cb;
	
	// wait for space in queue
	while (!QUEUE_Write(&ut_q,(uint8_t*)&transfer));
	
	usart_enableInterrupts(usart);
	
}
