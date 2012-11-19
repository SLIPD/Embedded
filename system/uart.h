#ifndef __UART_H__
#define __UART_H__

#include "efm32_usart.h"

void UART1_Init(uint8_t location);
void UART1_Send(uint8_t *data, uint16_t size);
void UART1_Recv(uint8_t *data, uint16_t size);

#endif