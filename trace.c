#include "trace.h"

#include "string.h"
#include "efm32_gpio.h"
#include "efm32_cmu.h"
#include "efm32_usart.h"

#include "led.h"

void TRACE_Init()
{
	
	#ifndef TRACE_OFF
	
	UART1->CLKDIV = 256 * ((CMU_ClockFreqGet(cmuClock_UART1)) / (9600 * 16) - 1);
	
	UART1->CMD = UART_CMD_TXEN | UART_CMD_RXEN | UART_CMD_CLEARRX | UART_CMD_CLEARTX;
	
	GPIO->P[4].MODEL = (GPIO->P[4].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_PUSHPULL;
	GPIO->P[4].MODEL = (GPIO->P[4].MODEL & ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_INPUT;
	
	UART1->ROUTE = UART_ROUTE_LOCATION_LOC3
          | UART_ROUTE_TXPEN | UART_ROUTE_RXPEN;
  
	#endif
	
}

void TRACE(char* msg)
{

	#ifndef TRACE_OFF
	
	LED_On(GREEN);
	
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	
	int todo, bytesToSend = strlen(msg);
	for (todo = 0; todo < bytesToSend; todo++) {
    while (!(UART1->STATUS & USART_STATUS_TXBL));
    UART1->TXDATA = *msg++;
  }
  
	while (!(UART1->STATUS & USART_STATUS_TXC));
	
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	
	LED_Off(GREEN);
	
	#endif
	
}