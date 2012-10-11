#ifndef __RADIO_H__
#define __RADIO_H__

#include <stdint.h>

#define RADIO_CE_PORT 1
#define RADIO_CE_PIN 1
#define RADIO_INT_PORT 1
#define RADIO_INT_PIN 0
#define RADIO_CSN_PORT 1
#define RADIO_CSN_PIN 2
#define RADIO_EN_RX_PORT 0
#define RADIO_EN_RX_PIN 2
#define RADIO_USART USART2
#define RADIO_USART_LOCATION USART_ROUTE_LOCATION_LOC1

#define RADIO_CSN_hi GPIO->P[RADIO_CSN_PORT].DOUTSET = (1 << RADIO_CSN_PIN);
#define RADIO_CSN_lo GPIO->P[RADIO_CSN_PORT].DOUTCLR = (1 << RADIO_CSN_PIN);
#define RADIO_CE_hi GPIO->P[RADIO_CE_PORT].DOUTSET = (1 << RADIO_CE_PIN);
#define RADIO_CE_lo GPIO->P[RADIO_CE_PORT].DOUTCLR = (1 << RADIO_CE_PIN);

#define RADIO_PACKET_SIZE 32

void RADIO_Init();
void RADIO_Main();
void RADIO_Interrupt();

void RADIO_Transmit(uint8_t packet[RADIO_PACKET_SIZE]);

#endif