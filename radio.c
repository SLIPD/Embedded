#include "radio.h"

#include "efm32_int.h"
#include "efm32_usart.h"
#include "efm32_timer.h"
#include "efm32_rtc.h"

#include <string.h>
#include <stdio.h>

#include "packets.h"
#include "usart.h"
#include "nRF24L01.h"
#include "led.h"
#include "trace.h"
#include "queue.h"
#include "time_schedule.h"

/* variables */
uint8_t txBufferMem[RADIO_BUFFER_SIZE * 32],
	rxBufferMem[RADIO_BUFFER_SIZE * 32];
queue_t txBuffer, rxBuffer;
bool auto_refil = false,
	send_in_progress = false;

uint8_t node_id = 0xFF;
uint32_t tdma_gp,
	tdma_txp,
	tdma_txp_p,
	tdma_sp,
	tdma_nc,
	tdma_p;

action_t radio_action1,
	radio_action2,
	radio_action3,
	radio_action4,
	radio_action5,
	radio_action6,
	radio_action7;
schedule_t radio_schedule;

/* prototypes */
void radio_stage1();
void radio_stage2();
void radio_stage3();
void radio_stage4();
void radio_stage5();
void radio_stage6();
void radio_stage7();

void radio_cs(USART_ChipSelect set);
uint8_t radio_readRegister(uint8_t reg);
void radio_writeRegister(uint8_t reg, uint8_t value);
void radio_config_cc(TIMER_TypeDef *timer, uint8_t cc, uint32_t time, bool high);

/* functions */
void GPIO_EVEN_IRQHandler()
{
	
	if (GPIO_IntGet() & (1 << NRF_INT_PIN))
	{
		
		uint8_t status = radio_readRegister(NRF_STATUS);
		radio_writeRegister(NRF_STATUS,0x70);
		GPIO_IntClear((1 << NRF_INT_PIN));
		
		// max rt
		if (status & 0x10)
		{
			
		}
		
		// tx
		if (status & 0x20)
		{
			
			TRACE("%i: radio_interrupt_rt() : TX\n", TIMER_CounterGet(TIMER1));
			
			uint8_t payload[33];
			
			int i = 0;
			while (auto_refil && (!(radio_readRegister(NRF_FIFO_STATUS) & 0x20)) && (QUEUE_Read(&txBuffer,&payload[1])))
			{
				
				payload[0] = NRF_W_TX_PAYLOAD;
				USART2_Transfer(payload,33,radio_cs);
				i++;
				
			}
			
			TRACE("%i: RADIO_TxBufferFill(): %i packets uploaded\n",TIMER_CounterGet(TIMER1), i);
			
			// if no packets left, send in progress false
			if (radio_readRegister(NRF_FIFO_STATUS) & 0x10)
				send_in_progress = false;
			else
				send_in_progress = true;
			
		}
		
		// rx
		if (status & 0x40)
		{
			
			TRACE("%i: radio_interrupt_rt() : RX\n", TIMER_CounterGet(TIMER1));
			uint8_t payload[33];
			
			while (!(radio_readRegister(NRF_FIFO_STATUS) & 0x01))
			{
				payload[0] = NRF_R_RX_PAYLOAD;
				USART2_Transfer(payload,33,radio_cs);
				
				QUEUE_Write(&rxBuffer, &payload[1]);
				
			}
			
		}
		
	}
	
}

void RADIO_Init()
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
	
	TRACE("RADIO INIT [STATUS = %2.2X]\n", radio_readRegister(NRF_STATUS));
	
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
	
	TRACE("%i: RADIO_SetMode(): set mode to: 0x%2.2X\n", TIMER_CounterGet(TIMER1), rm);
	
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

	TRACE("%i: RADIO_Enable(): enable: 0x%2.2X\n", TIMER_CounterGet(TIMER1), rm);

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
		case RXAMP:
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
	
	TRACE("%i: RADIO_TxBufferFill(): %i packets uploaded\n",TIMER_CounterGet(TIMER1), i);
	
	if (radio_readRegister(NRF_FIFO_STATUS) & 0x10)
				send_in_progress = false;
			else
				send_in_progress = true;
	
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

void RADIO_GetID()
{
	
	Packet ident, incoming;
	
	ident.originId = 0xFF;
	ident.destinationId = 0x00;
	ident.ttl = 1;
	ident.msgType = 0x00;
	
	ident.payload.identification.id0 = *((uint32_t*)(0xFE081F0));
	ident.payload.identification.id1 = *((uint32_t*)(0xFE081F4));
	ident.payload.identification.nodeId = 0xFF;
	
	int32_t next_send = 0,
		last = RTC_CounterGet();
	
	RADIO_Enable(OFF);
	RADIO_SetMode(RX);
	RADIO_Enable(RX);
	
	bool identified = false;
	while(!identified)
	{
		
		if (RTC_CounterGet() > next_send)
		{
			RADIO_Enable(OFF);
			RADIO_SetMode(TX);
			RADIO_Send((uint8_t*)&ident);
			RADIO_TxBufferFill();
			RADIO_Enable(TX);
			while (RADIO_Sending());
			RADIO_Enable(OFF);
			RADIO_SetMode(RX);
			RADIO_Enable(RX);
			
			if (next_send < RTC_CounterGet() + 32768)
				next_send = RTC_CounterGet() + 32768;
		}
		
		while (RADIO_Recv((uint8_t*)&incoming))
		{
			
			if (incoming.originId == 0x00 &&
				incoming.destinationId == 0xFF &&
				incoming.msgType == 0x00 &&
				incoming.payload.identification.id0 == *((uint32_t*)(0xFE081F0)) &&
				incoming.payload.identification.id1 == *((uint32_t*)(0xFE081F4)))
			{
				
				// store tdma details
				node_id = incoming.payload.identification.nodeId;
				
				/*
				tdma_gp = ;
				tdma_txp = ;
				tdma_txp_p = ;
				tdma_nc = ;
				
				tdma_sp = 2*tdma_gp + tdma_txp;
				tdma_p = tdma_sp * tdma_nc;
				*/
				
				identified = true;
				
				break;
				
			}
			
			if (next_send < RTC_CounterGet() + 3277 + (*((uint32_t*)(0xFE081F0)) & 0xFF))
				next_send = RTC_CounterGet() + 3277 + (*((uint32_t*)(0xFE081F0)) & 0xFF);
			
		}
		
		if (last > RTC_CounterGet())
			next_send -= 32768;
		
		last = RTC_CounterGet();
		
	}
	
	TRACE("IDENTIFIED NODE ID:%i\n", node_id);
	
}

void RADIO_EnableTDMA()
{
	
	TS_Init(&radio_schedule, RADIO_TIMER, SCHEDULE_TIMER_CC);
	
	radio_action1.time = 1;
	radio_action1.action = radio_stage1;
	TS_Insert(&radio_schedule, &radio_action1);
	
	radio_action2.time = node_id * tdma_sp + 1;
	radio_action2.action = radio_stage2;
	TS_Insert(&radio_schedule, &radio_action2);
	
	radio_action3.time = node_id * tdma_sp + tdma_gp + 1;
	radio_action3.action = radio_stage3;
	TS_Insert(&radio_schedule, &radio_action3);
	
	radio_action4.time = node_id * tdma_sp + tdma_gp + tdma_txp - tdma_txp_p;
	radio_action4.action = radio_stage4;
	TS_Insert(&radio_schedule, &radio_action4);
	
	radio_action5.time = node_id * tdma_sp + tdma_gp + tdma_txp;
	radio_action5.action = radio_stage5;
	TS_Insert(&radio_schedule, &radio_action5);
	
	radio_action6.time = (node_id + 1)  * tdma_sp + 1;
	radio_action6.action = radio_stage6;
	TS_Insert(&radio_schedule, &radio_action6);
	
	radio_action7.time = tdma_p;
	radio_action7.action = radio_stage7;
	TS_Insert(&radio_schedule, &radio_action7);
	
	// configure timer
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true, 
		.debugRun   = true, 
		.prescale   = timerPrescale1024, 
		.clkSel     = timerClkSelHFPerClk, 
		.fallAction = timerInputActionNone, 
		.riseAction = timerInputActionNone, 
		.mode       = timerModeUp, 
		.dmaClrAct  = false,
		.quadModeX4 = false, 
		.oneShot    = false, 
		.sync       = false, 
	};
	
	TIMER_Reset(RADIO_TIMER);
	TIMER_TopSet(RADIO_TIMER, (48000000 / 1024));
	TIMER_Init(RADIO_TIMER, &timerInit);
	
	TIMER_IntEnable(RADIO_TIMER, SCHEDULE_TIMER_IRQ);
	
	// enable TS
	TS_Complete(&radio_schedule);
	
}

void RADIOTIMER_IRQHandler()
{
	
	if (TIMER_IntGet(RADIO_TIMER) & SCHEDULE_TIMER_IRQ)
	{
		TS_Update(&radio_schedule);
		
		TIMER_IntClear(RADIO_TIMER, SCHEDULE_TIMER_IRQ);
	}
	
}

void radio_config_cc(TIMER_TypeDef *timer, uint8_t cc, uint32_t time, bool set)
{
	
	TIMER_InitCC_TypeDef timerCCInit = 
	{
		.cufoa      = timerOutputActionNone,
		.cofoa      = timerOutputActionNone,
		.cmoa       = timerOutputActionNone,
		.mode       = timerCCModeCompare,
		.filter     = true,
		.prsInput   = false,
		.coist      = false,
		.outInvert  = false,
	};
	
	if (set)
	{
		timerCCInit.cmoa = timerOutputActionSet;
	}
	else
	{
		timerCCInit.cmoa = timerOutputActionClear;
	}
	
	TIMER_CompareSet(timer,cc, time);
	TIMER_InitCC(timer, cc, &timerCCInit);
	
}

void radio_stage1()
{
	
	radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, node_id * tdma_sp, false);
	RADIO_Enable(RXAMP);
	
}

void radio_stage2()
{
	
	RADIO_Enable(OFF);
	RADIO_SetMode(TX);
	
	radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, node_id * tdma_sp + tdma_gp, true);
	
	RADIO_TxBufferFill();
	RADIO_SetAutoRefil(true);
	
}

void radio_stage3() 
{
	
	radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, node_id * tdma_sp + tdma_gp + tdma_txp, false);
	
}

void radio_stage4()
{
	
	RADIO_SetAutoRefil(false);
	
}

void radio_stage5()
{
	
	RADIO_SetMode(RX);
	radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, (node_id + 1) * tdma_sp, true);
	
}

void radio_stage6()
{
	
	radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, tdma_p, false);
	RADIO_Enable(RXAMP);
	
}

void radio_stage7()
{
	
	RADIO_Enable(OFF);
	radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, 0, true);
	
}
