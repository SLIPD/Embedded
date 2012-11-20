#ifndef __RADIO_TASK_H__
#define __RADIO_TASK_H__

#include <stdint.h>
#include <stdbool.h>

#include "efm32_gpio.h"

#define NRF_CE_PORT 1
#define NRF_CE_PIN 1
#define NRF_INT_PORT 1
#define NRF_INT_PIN 0
#define NRF_CSN_PORT 1
#define NRF_CSN_PIN 2
#define NRF_RXEN_PORT 0
#define NRF_RXEN_PIN 2

#define NRF_CE_lo GPIO_PinOutClear(NRF_CE_PORT, NRF_CE_PIN)
#define NRF_CE_hi GPIO_PinOutSet(NRF_CE_PORT, NRF_CE_PIN)

#define NRF_CSN_lo GPIO_PinOutClear(NRF_CSN_PORT, NRF_CSN_PIN)
#define NRF_CSN_hi GPIO_PinOutSet(NRF_CSN_PORT, NRF_CSN_PIN)

#define NRF_RXEN_lo GPIO_PinOutClear(NRF_RXEN_PORT, NRF_RXEN_PIN)
#define NRF_RXEN_hi GPIO_PinOutSet(NRF_RXEN_PORT, NRF_RXEN_PIN)

#define RADIO_BUFFER_SIZE 		16

typedef enum 
{
	OFF,
	TX,
	RX
} RADIO_Mode;

bool RADIO_Send(uint8_t payload[32]);
bool RADIO_Recv(uint8_t payload[32]);
void RADIO_SetMode(RADIO_Mode rm);
void RADIO_Enable(RADIO_Mode rm);
void RADIO_TxBufferFill();
void RADIO_SetAutoRefil(bool auto_refil);
uint16_t RADIO_TxBufferSize();
uint16_t RADIO_RxBufferSize();
bool RADIO_Sending();

#endif