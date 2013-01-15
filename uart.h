#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#define UART_BUFFER_SIZE 10240

void UART1_Init(uint8_t location);
void UART1_Send(uint8_t *data, uint16_t size);
void UART1_SetRecvHandler(void (*uart_handler)(uint8_t));
void UART1_ClearRecvHandler();

#endif