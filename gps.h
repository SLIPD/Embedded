#ifndef __GPS_H__
#define __GPS_H__

#include "stdint.h"

#include "efm32_usart.h"

#define GPS_UART UART0
#define GPS_UART_TX_PORT 0
#define GPS_UART_TX_PIN 3
#define GPS_UART_RX_PORT 0
#define GPS_UART_RX_PIN 4

#define GPS_WAKEUP_PORT 4
#define GPS_WAKEUP_PIN 12
#define GPS_ONOFF_PORT 4
#define GPS_ONOFF_PIN 13
#define GPS_TM_PORT 4
#define GPS_TM_PIN 14

void GPS_Init();
void GPS_Read(uint8_t packet[]);

#endif