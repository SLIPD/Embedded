#include "radio.h"

#include "efm32_int.h"
#include "efm32_usart.h"
#include "efm32_timer.h"
#include "efm32_rtc.h"
#include "efm32_int.h"

#include <string.h>
#include <stdio.h>

#include "packets.h"
#include "usart.h"
#include "nRF24L01.h"
#include "led.h"
#include "trace.h"
#include "queue.h"
#include "time_schedule.h"
#include "config.h"

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
	tdma_p,
	tdma_c;

action_t radio_action1,
	radio_action2,
	radio_action3,
	radio_action4,
	radio_action5,
	radio_action6,
	radio_action7,
	radio_action8;
schedule_t radio_schedule;

volatile uint16_t tdma_stage_flags = 0;
volatile uint8_t radio_irq_flags = 0;
uint16_t tx_packet_count = 0,
	rx_packet_count = 0;
uint32_t last_tx = 0;

bool wait_callback = false;
uint8_t readRegisterValue;

bool syncd = false;

/* prototypes */
void radio_stage1();
void radio_stage2();
void radio_stage3();
void radio_stage4();
void radio_stage5();
void radio_stage6();
void radio_stage7();
void radio_stage8();

void radio_cs(USART_ChipSelect set);
uint8_t radio_readRegister(uint8_t reg);
void radio_writeRegister(uint8_t reg, uint8_t value);
void radio_config_cc(TIMER_TypeDef *timer, uint8_t cc, uint32_t time, bool high);
void radio_readRegisterCB(uint8_t *data, uint16_t size);
void radio_storePacket(uint8_t *data, uint16_t size);

/* functions */
void GPIO_EVEN_IRQHandler()
{
	
	if (GPIO_IntGet() & (1 << NRF_INT_PIN))
	{
		
		TRACE("%i: RADIO IRQ\n", TIMER_CounterGet(TIMER1));
		if (USART_Ready())
		{
			radio_irq_flags = radio_readRegister(NRF_STATUS);
			radio_writeRegister(NRF_STATUS,0x70);
		}
		GPIO_IntClear((1 << NRF_INT_PIN));
		
	}
	
}

void radio_readRegisterCB(uint8_t *data, uint16_t size)
{
	readRegisterValue = data[1];
	wait_callback = false;
}

void RADIO_Init()
{
	
	TIMER_Reset(RADIO_TIMER); 
	TIMER_Reset(PPS_TIMER);
	
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
	USART2_Transfer(addr,6,radio_cs, NULL);

	memset(addr,0xE7,6);
	addr[0] = 0x00 | (NRF_W_REGISTER | NRF_RX_ADDR_P0);
	USART2_Transfer(addr,6,radio_cs, NULL);

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
	USART2_Transfer(&cmd,1,radio_cs, NULL);
	cmd = NRF_FLUSH_TX;
	USART2_Transfer(&cmd,1,radio_cs, NULL);
	
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
	wait_callback = true;
	USART2_Transfer(transfer,2,radio_cs, radio_readRegisterCB);
	while(wait_callback);
	return readRegisterValue;
}

void radio_writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t transfer[2];
	transfer[0] = (NRF_W_REGISTER | reg);
	transfer[1] = value;
	USART2_Transfer(transfer,2,radio_cs, NULL);
}

bool RADIO_Send(uint8_t payload[32])
{
	
	return QUEUE_Write(&txBuffer, payload);
	
}

bool RADIO_Recv(uint8_t payload[32])
{
	
	return QUEUE_Read(&rxBuffer, payload);
	
}

void radio_storePacket(uint8_t *data, uint16_t size)
{
	
	Packet *p = (Packet*)&data[1];
	#ifdef BASESTATION
		
		if (p->destinationId == 0x00 || p->destinationId == 0xFF)
		{
			QUEUE_Write(&rxBuffer, &data[1]);
		}
		return;
		
	#endif
	
	rx_packet_count++;
	
	if (p->destinationId == node_id)
	{
		QUEUE_Write(&rxBuffer, &data[1]);
		return;
	}
	
	if (p->destinationId == 0xFF)
	{
		QUEUE_Write(&rxBuffer, &data[1]);
	}
	
	if (--p->ttl == 0)
		return;
	
	QUEUE_Write(&txBuffer, &data[1]);
	
}

void RADIO_SetAutoRefil(bool _auto_refil)
{
	auto_refil = _auto_refil;
}

void RADIO_TxBufferFill()
{
	
	TRACE("%i: TX BUFFER FILL\n", TIMER_CounterGet(TIMER1));
	
	if (radio_readRegister(NRF_FIFO_STATUS) & 0x10)
		send_in_progress = false;
	
	uint8_t payload[33];
	if ((radio_readRegister(NRF_FIFO_STATUS) & 0x10))
	{
		int i;
		for (i = 0; i < 1; i++)
		{
			if (!QUEUE_Read(&txBuffer,&payload[1]))
				break;
			payload[0] = NRF_W_TX_PAYLOAD;
			USART2_Transfer(payload,33,radio_cs, NULL);
			
			send_in_progress = true;
		}
		TRACE("%i: RADIO_TxBufferFill(): %i packets uploaded\n",TIMER_CounterGet(TIMER1), i);
		tx_packet_count += i;
	}
	else if ((!(radio_readRegister(NRF_FIFO_STATUS) & 0x20)) && (QUEUE_Read(&txBuffer,&payload[1])))
	{
		
		payload[0] = NRF_W_TX_PAYLOAD;
		USART2_Transfer(payload,33,radio_cs, NULL);
		
		send_in_progress = true;
		
		TRACE("%i: RADIO_TxBufferFill(): 1 packet uploaded\n",TIMER_CounterGet(TIMER1));
		tx_packet_count++;
	}
	
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
	
	TRACE("GET ID\n");
	
	bool identified = false;
	while(!identified)
	{
		
		RADIO_HandleMessages();
		
		if (RTC_CounterGet() > next_send)
		{
			RADIO_Enable(OFF);
			RADIO_SetMode(TX);
			RADIO_Send((uint8_t*)&ident);
			RADIO_TxBufferFill();
			//for (int i = 0; i < 10000; i++);
			RADIO_Enable(TX);
			while(RADIO_Sending())
			{
				RADIO_HandleMessages();
			}
			RADIO_Enable(OFF);
			RADIO_SetMode(RX);
			RADIO_Enable(RX);
			
			if (next_send < RTC_CounterGet() + 32768)
				next_send = RTC_CounterGet() + 32768;
		}
		
		while (RADIO_Recv((uint8_t*)&incoming))
		{
			
			/*
			uint8_t* pos = (uint8_t*)&incoming;
			int i;
			for (i = 0; i < 32; i++)
			{
				TRACE("0x%2.2X ", *pos++);
			}
			TRACE("\n");
			*/
				
			if (incoming.originId == 0x00 &&
				incoming.destinationId == 0xFF &&
				incoming.msgType == 0x00 &&
				incoming.payload.identification.id0 == *((uint32_t*)(0xFE081F0)) &&
				incoming.payload.identification.id1 == *((uint32_t*)(0xFE081F4)))
			{
				
				// store tdma details
				node_id = incoming.payload.identification.nodeId;
				
				tdma_gp = incoming.payload.identification.tdma_gp;
				tdma_txp = incoming.payload.identification.tdma_txp;
				tdma_txp_p = incoming.payload.identification.tdma_txp_p;
				tdma_nc = incoming.payload.identification.nc;
				tdma_c = incoming.payload.identification.c;
				
				tdma_sp = 2*tdma_gp + tdma_txp;
				tdma_p = tdma_sp * tdma_nc;
				
				identified = true;
				
				break;
				
			}
			
			if (next_send < RTC_CounterGet() + 3277 + (*((uint32_t*)(0xFE081F0)) & 0xFF))
				next_send = RTC_CounterGet() + 3277 + (*((uint32_t*)(0xFE081F0)) & 0xFF);
			
		}
		
		if (last > RTC_CounterGet())
			next_send -= 32768;
		
		last = RTC_CounterGet();
		RADIO_HandleMessages();
		
	}
	
	TRACE("IDENTIFIED NODE ID:%i\n", node_id);
	
	TRACE("TDMA CONFIG\n");
				TRACE("GP: %i\nTXP: %i\nTXP_P: %i\nNC: %i\nC: \nSP: %i\nP: %i\n",
					tdma_gp,
					tdma_txp,
					tdma_txp_p,
					tdma_nc,
					tdma_c,
					tdma_sp,
					tdma_p);
	
}

void RADIO_EnableTDMA()
{
	
	TIMER_TopSet(PPS_TIMER, (48000000 / 1024));
	TIMER_TopSet(RADIO_TIMER, (48000000 / 1024));
	
	// set up capture pin for tdma
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

	TIMER_Init(PPS_TIMER, &timerInit);
	TIMER_Init(RADIO_TIMER, &timerInit);

	// enable CC for timer sync
	TIMER_InitCC_TypeDef timerCCInitCapture = 
	{
		.eventCtrl  = timerEventRising,
		.edge       = timerEdgeRising,
		.cufoa      = timerOutputActionNone,
		.cofoa      = timerOutputActionNone,
		.cmoa       = timerOutputActionNone,
		.mode       = timerCCModeCapture,
		.filter     = true,
		.prsInput   = false,
		.coist      = false,
		.outInvert  = false,
	};

	PPS_TIMER->ROUTE |= PPS_TIMER_CC_ROUTE;
	
	GPIO_PinModeSet(PPS_PORT, PPS_PIN, gpioModeInput, 0);
	
	TIMER_IntEnable(PPS_TIMER, PPS_TIMER_IRQ);
	TIMER_InitCC(PPS_TIMER, PPS_TIMER_CC, &timerCCInitCapture);
	
	while (!syncd);
	
	TRACE("Syncd\n");
	TRACE("Switching to channel %i\n", tdma_c);
	radio_writeRegister(NRF_RF_CH,tdma_c);
	
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
	
	radio_action8.time = TIMER_TopGet(RADIO_TIMER) - tdma_gp*2;
	radio_action8.action = radio_stage8;
	TS_Insert(&radio_schedule, &radio_action8);
	
	TRACE("ACTION 1: %i\n", radio_action1.time);
	TRACE("ACTION 2: %i\n", radio_action2.time);
	TRACE("ACTION 3: %i\n", radio_action3.time);
	TRACE("ACTION 4: %i\n", radio_action4.time);
	TRACE("ACTION 5: %i\n", radio_action5.time);
	TRACE("ACTION 6: %i\n", radio_action6.time);
	TRACE("ACTION 7: %i\n", radio_action7.time);
	TRACE("ACTION 8: %i\n", radio_action8.time);
	
	RADIO_TIMER->ROUTE |= NRF_CE_TIMER_CC_ROUTE;
	
	// configure timer
	
	TRACE("TIMER TOP: %i\n", TIMER_TopGet(RADIO_TIMER));
	
	// enable TS
	TS_Complete(&radio_schedule);
	
	INT_Disable();
	
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	TIMER_IntClear(RADIO_TIMER, SCHEDULE_TIMER_IRQ);
	TIMER_IntEnable(RADIO_TIMER, SCHEDULE_TIMER_IRQ);
	
	INT_Enable();
	
}

void RADIOTIMER_IRQHandler()
{
	
	uint32_t flags = TIMER_IntGet(RADIO_TIMER);
	
	if (flags & SCHEDULE_TIMER_IRQ)
	{
		TS_Update(&radio_schedule);
		
		TIMER_IntClear(RADIO_TIMER, SCHEDULE_TIMER_IRQ);
	}
	
	TIMER_IntClear(RADIO_TIMER, TIMER_IntGet(RADIO_TIMER));
	
}

void PPSTIMER_IRQHandler()
{
	
	if (TIMER_IntGet(PPS_TIMER) & PPS_TIMER_IRQ)
	{
		uint32_t diff = TIMER_CounterGet(TIMER0) - TIMER_CaptureGet(TIMER0,PPS_TIMER_CC);
		TIMER_CounterSet(TIMER0,diff);
		TIMER_CounterSet(TIMER1,diff);
		TIMER_IntClear(PPS_TIMER, PPS_TIMER_IRQ);
		syncd = true;
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
		TRACE("%i: SET @ %i\n", TIMER_CounterGet(TIMER1), time);
	}
	else
	{
		timerCCInit.cmoa = timerOutputActionClear;
		TRACE("%i: CLEAR @ %i\n", TIMER_CounterGet(TIMER1), time);
	}
	
	TIMER_CompareSet(timer,cc, time);
	TIMER_InitCC(timer, cc, &timerCCInit);
	
}

void radio_stage1()
{
	tdma_stage_flags |= (1 << 0);
}

void radio_stage2()
{
	tdma_stage_flags |= (1 << 1);
}

void radio_stage3() 
{
	tdma_stage_flags |= (1 << 2);
}

void radio_stage4()
{
	RADIO_SetAutoRefil(false);
	tdma_stage_flags |= (1 << 3);
}

void radio_stage5()
{
	tdma_stage_flags |= (1 << 4);
}

void radio_stage6()
{
	tdma_stage_flags |= (1 << 5);
}

void radio_stage7()
{
	tdma_stage_flags |= (1 << 6);
}

void radio_stage8()
{
	tdma_stage_flags |= (1 << 7);
}

void RADIO_HandleMessages()
{
	
	if (tdma_stage_flags & (1 << 0))
	{
		TRACE("%i: STAGE 1 - RX EN\n", TIMER_CounterGet(TIMER1));
		radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, node_id * tdma_sp, false);
		
		RADIO_Enable(RXAMP);
		tdma_stage_flags &= ~(1 << 0);
	}
	if (tdma_stage_flags & (1 << 1))
	{
		TRACE("%i: STAGE 2 - CONF TX, FILL FIFO\n", TIMER_CounterGet(TIMER1));
		radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, node_id * tdma_sp + tdma_gp, true);
		
		RADIO_Enable(OFF);
		TRACE("%i packets received\n", rx_packet_count);
		rx_packet_count = 0;
		
		RADIO_SetMode(TX);
		tx_packet_count = 0;
		RADIO_TxBufferFill();
		RADIO_SetAutoRefil(true);
		tdma_stage_flags &= ~(1 << 1);
	}
	if (tdma_stage_flags & (1 << 2))
	{
		TRACE("%i: STAGE 3\n", TIMER_CounterGet(TIMER1));
		radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, node_id * tdma_sp + tdma_gp + tdma_txp, false);
		tdma_stage_flags &= ~(1 << 2);
	}
	if (tdma_stage_flags & (1 << 3))
	{
		TRACE("%i: STAGE 4 - AUTO REFIL = FALSE\n", TIMER_CounterGet(TIMER1));
		tdma_stage_flags &= ~(1 << 3);
	}
	if (tdma_stage_flags & (1 << 4))
	{
		TRACE("%i: STAGE 5 - CONF RX\n", TIMER_CounterGet(TIMER1));
		radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, (node_id + 1) * tdma_sp, true);
		TRACE("%i packets sent, last TX %i\n", tx_packet_count, last_tx);
		RADIO_SetMode(RX);
		tdma_stage_flags &= ~(1 << 4);
	}
	if (tdma_stage_flags & (1 << 5))
	{
		TRACE("%i: STAGE 6 - ENABLE RX\n", TIMER_CounterGet(TIMER1));
		radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, tdma_p, false);
		RADIO_Enable(RXAMP);
		tdma_stage_flags &= ~(1 << 5);
	}
	if (tdma_stage_flags & (1 << 6))
	{
		TRACE("%i: STAGE 7 - DISABLE\n", TIMER_CounterGet(TIMER1));
		radio_config_cc(RADIO_TIMER, NRF_CE_TIMER_CC, 0, true);
		RADIO_Enable(OFF);
		RADIO_SetMode(OFF);
		tdma_stage_flags &= ~(1 << 6);
	}
	if (tdma_stage_flags & (1 << 7))
	{
		TRACE("%i: STAGE 8 - CONF RX\n", TIMER_CounterGet(TIMER1));
		RADIO_SetMode(RX);
		tdma_stage_flags &= ~(1 << 7);
	}
	
	if (radio_irq_flags)
	{
		
		uint8_t status = radio_irq_flags;
		radio_irq_flags = 0;
			
		// max rt
		if (status & 0x10)
		{
			
		}
		
		// tx
		if (status & 0x20)
		{
			NRF_CE_lo;
			
			TRACE("%i: radio_interrupt_rt() : TX\n", TIMER_CounterGet(TIMER1));
			last_tx = TIMER_CounterGet(TIMER1);
			
			if (radio_readRegister(NRF_FIFO_STATUS) & 0x10)
				send_in_progress = false;
			
			uint8_t payload[33];
			if (auto_refil && (radio_readRegister(NRF_FIFO_STATUS) & 0x10))
			{
				int i;
				for (i = 0; i < 1; i++)
				{
					if (!QUEUE_Read(&txBuffer,&payload[1]))
						break;
					payload[0] = NRF_W_TX_PAYLOAD;
					USART2_Transfer(payload,33,radio_cs, NULL);
					
					send_in_progress = true;
				}
				TRACE("%i: RADIO_TxBufferFill(): %i packets uploaded\n",TIMER_CounterGet(TIMER1), i);
				tx_packet_count += i;
			}
			else if (auto_refil && (!(radio_readRegister(NRF_FIFO_STATUS) & 0x20)) && (QUEUE_Read(&txBuffer,&payload[1])))
			{
				
				payload[0] = NRF_W_TX_PAYLOAD;
				USART2_Transfer(payload,33,radio_cs, NULL);
				
				send_in_progress = true;
				
				TRACE("%i: RADIO_TxBufferFill(): 1 packet uploaded\n",TIMER_CounterGet(TIMER1));
				tx_packet_count++;
			}
			
			NRF_CE_hi;
			
		}
		
		// rx
		if (status & 0x40)
		{
			
			TRACE("%i: radio_interrupt_rt() : RX\n", TIMER_CounterGet(TIMER1));
			
			uint8_t payload[33],
				fifo_status = radio_readRegister(NRF_FIFO_STATUS);
			if (fifo_status & 0x02)
			{
				for (int i = 0; i < 3; i++)
				{
					payload[0] = NRF_R_RX_PAYLOAD;
					USART2_Transfer(payload,33,radio_cs,radio_storePacket);
				}
			} else if (!(fifo_status & 0x01))
			{
				payload[0] = NRF_R_RX_PAYLOAD;
				USART2_Transfer(payload,33,radio_cs,radio_storePacket);
			}
			
		}
		
	}
	
}
