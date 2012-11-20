#include "radio_init_task.h"

#include "efm32_int.h"
#include "efm32_usart.h"

#include <string.h>

#include "config.h"
#include "packets.h"
#include "tasks.h"
#include "system.h"

#include "nRF24L01.h"
#include "led.h"

/* variables */
uint8_t txBufferMem[RADIO_BUFFER_SIZE * 32],
	rxBufferMem[RADIO_BUFFER_SIZE * 32];
queue_t txBuffer, rxBuffer;
bool auto_refil = false,
	send_in_progress = false;

/* prototypes */
void radio_cs(USART_ChipSelect set);
uint8_t radio_readRegister(uint8_t reg);
void radio_writeRegister(uint8_t reg, uint8_t value);
void radio_interrupt_rt();

/* functions */
void GPIO_EVEN_IRQHandler()
{
	
	if (GPIO_IntGet() & (1 << NRF_INT_PIN))
	{
		
		SCHEDULER_RunRTTask(&radio_interrupt_rt);
		
		GPIO_IntClear((1 << NRF_INT_PIN));
		
	}
	
}

void radio_interrupt_rt()
{
	
	uint8_t status = radio_readRegister(NRF_STATUS);
	
	// max rt
	if (status & 0x10)
	{
		
	}
	
	// tx
	if (status & 0x20)
	{
		
		char tmsg[255];
		sprintf(tmsg, "%i: radio_interrupt_rt() : TX\n", TIMER_CounterGet(TIMER1));
		TRACE(tmsg);
		
		if (auto_refil)
		{
			RADIO_TxBufferFill();
		}
		
		// if no packets left, send in progress false
		if (radio_readRegister(NRF_FIFO_STATUS) & 0x10)
			send_in_progress = false;
		
	}
	
	// rx
	if (status & 0x40)
	{
		
		char tmsg[255];
		sprintf(tmsg, "%i: radio_interrupt_rt() : RX\n", TIMER_CounterGet(TIMER1));
		TRACE(tmsg);
		
		// store all received packets
		uint8_t payload[33];
		int i = 0;
		while (!(radio_readRegister(NRF_FIFO_STATUS) & 0x01))
		{
			payload[0] = NRF_R_RX_PAYLOAD;
			USART2_Transfer(payload,33,radio_cs);
			
				
			QUEUE_Write(&rxBuffer, &payload[1]);
			
			i++;
		}
		
	}
	
	radio_writeRegister(NRF_STATUS,0x70);
	
}

void radio_init_task_entrypoint()
{
	
	QUEUE_Init(&txBuffer, txBufferMem, 32, RADIO_BUFFER_SIZE);
	QUEUE_Init(&rxBuffer, rxBufferMem, 32, RADIO_BUFFER_SIZE);
	
	GPIO_PinModeSet(NRF_CE_PORT, NRF_CE_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(NRF_CSN_PORT, NRF_CSN_PIN, gpioModePushPull, 1);
	GPIO_PinModeSet(NRF_RXEN_PORT, NRF_RXEN_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(NRF_INT_PORT, NRF_INT_PIN, gpioModeInput, 0);
	
	GPIO_PinModeSet(gpioPortB, 5, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortB, 4, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortB, 3, gpioModePushPull, 0);
	
	USART2_Init(1);
	
	// configure gpio interrupt
	radio_writeRegister(NRF_STATUS,0x70);
	GPIO_IntClear(1 << NRF_INT_PIN);
	GPIO_IntConfig(NRF_INT_PORT,NRF_INT_PIN,false,true,true);
	
	radio_writeRegister(NRF_EN_AA,0x00);
	radio_writeRegister(NRF_EN_RXADDR,0x3F);
	radio_writeRegister(NRF_SETUP_AW,0x03);
	radio_writeRegister(NRF_SETUP_RETR,0x00);
	radio_writeRegister(NRF_RF_CH,2);
	radio_writeRegister(NRF_RF_SETUP,0x0F);
	
	radio_writeRegister(NRF_RX_PW_P0,32);
	
	uint8_t addr[6];
	memset(addr,0xE7,6);
	addr[0] = 0x00 | (NRF_W_REGISTER | NRF_TX_ADDR);
	USART2_Transfer(addr,6,radio_cs);
	
	memset(addr,0xE7,6);
	addr[0] = 0x00 | (NRF_W_REGISTER | NRF_RX_ADDR_P0);
	USART2_Transfer(addr,6,radio_cs);
	
	radio_writeRegister(NRF_DYNPD, 0x00);
	radio_writeRegister(NRF_FEATURE, 0x00);
	
	RADIO_SetMode(OFF);
	RADIO_Enable(OFF);
	
	SCHEDULER_TaskInit(&tdma_setup_task, tdma_setup_task_entrypoint);
	
	#ifdef BASESTATION
		
		SCHEDULER_TaskInit(&basestation_radio_task, basestation_radio_task_entrypoint);
		
	#else
	
		SCHEDULER_TaskInit(&node_radio_task, node_radio_task_entrypoint);
	
	#endif
	
}

void radio_cs(USART_ChipSelect set)
{
	
	switch (set)
	{
	case HIGH:
		NRF_CSN_hi;
		break;
	case LOW:
		NRF_CSN_lo;
		break;
	}
	
}

void RADIO_SetMode(RADIO_Mode rm)
{
	
	char tmsg[255];
	sprintf(tmsg,"RADIO_SetMode(): set mode to: 0x%2.2X\n", rm);
	TRACE(tmsg);
	
	uint8_t cmd = NRF_FLUSH_RX;
	USART2_Transfer(&cmd,1,radio_cs);
	cmd = NRF_FLUSH_TX;
	USART2_Transfer(&cmd,1,radio_cs);
	
	switch (rm)
	{
		default:
		case OFF:
			radio_writeRegister(NRF_CONFIG, 0x0C); LED_Off(GREEN);LED_Off(BLUE);
			break;
		case TX:
			radio_writeRegister(NRF_CONFIG, 0x0E); LED_On(GREEN);LED_Off(BLUE);
			break;
		case RX:
			radio_writeRegister(NRF_CONFIG, 0x0F); LED_Off(GREEN);LED_On(BLUE);
			break;
	}
}

void RADIO_Enable(RADIO_Mode rm)
{
	switch (rm)
	{
		default:
		case OFF:
			NRF_RXEN_lo;
			NRF_CE_lo;
			break;
		case TX:
			NRF_RXEN_lo;
			NRF_CE_hi;
			break;
		case RX:
			NRF_CE_hi;
			NRF_RXEN_hi;
			break;
	}
}

uint8_t radio_readRegister(uint8_t reg)
{
	uint8_t transfer[2];
	transfer[0] = (NRF_R_REGISTER | reg);
	transfer[1] = NRF_NOP;
	USART2_Transfer(transfer,2,radio_cs);
	return transfer[1];
}

void radio_writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t transfer[2];
	transfer[0] = (NRF_W_REGISTER | reg);
	transfer[1] = value;
	USART2_Transfer(transfer,2,radio_cs);
}

bool RADIO_Send(uint8_t payload[32])
{
	
	return QUEUE_Write(&txBuffer, payload);
	
}

bool RADIO_Recv(uint8_t payload[32])
{
	
	return QUEUE_Read(&rxBuffer, payload);
	
}

void RADIO_SetAutoRefil(bool _auto_refil)
{
	auto_refil = _auto_refil;
}

void RADIO_TxBufferFill()
{
	
	uint8_t payload[33];
	
	int i = 0;
	while ((!(radio_readRegister(NRF_FIFO_STATUS) & 0x20)) && (QUEUE_Read(&txBuffer,&payload[1])))
	{
		
		payload[0] = NRF_W_TX_PAYLOAD;
		USART2_Transfer(payload,33,radio_cs);
		i++;
		
	}
	
	if (i > 0)
		send_in_progress = true;
	
	char tmsg[255];
	sprintf(tmsg, "%i: RADIO_TxBufferFill(): fill tx buffer (total pushed: %i)\n", TIMER_CounterGet(TIMER0), i);
	TRACE(tmsg);
	
}

uint16_t RADIO_TxBufferSize()
{
	
	return QUEUE_Fill(&txBuffer);
	
}

uint16_t RADIO_RxBufferSize()
{
	
	return QUEUE_Fill(&rxBuffer);
	
}

bool RADIO_Sending()
{
	
	return send_in_progress;
	
}
