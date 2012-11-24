#include "usart.h"

#include <stdbool.h>

#include "efm32_int.h"
#include "efm32_usart.h"

#include "led.h"

/* variables */

/* prototypes */

/* functions */
void USART_Init(USART_TypeDef *usart, uint8_t usart_idx, uint8_t location)
{
	
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
	usartInit.baudrate = 1000000;
	USART_InitSync(usart, &usartInit);
	usart->ROUTE = (usart->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | usart_location;
	usart->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;
	
}

void USART_Transfer(USART_TypeDef *usart, uint8_t usart_idx, uint8_t *buffer, uint16_t size, void (*cs)(USART_ChipSelect))
{
	
	volatile uint8_t scratch;
	
	while(!(usart->STATUS & USART_STATUS_TXBL));
	while(usart->STATUS & USART_STATUS_RXDATAV)
		scratch = usart->RXDATA;
	
	cs(LOW);
	
	int i;
	for (i = 0; i < size; i++)
	{
		usart->TXDATA = buffer[i];
		while (!(usart->STATUS & USART_STATUS_TXC));
		buffer[i] = usart->RXDATA;
	}
	
	cs(HIGH);
	
}
