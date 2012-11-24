//#define PPS_MASTER

#include "efm32.h"

#include "efm32_chip.h"
#include "efm32_rtc.h"
#include "efm32_gpio.h"
#include "efm32_cmu.h"
#include "efm32_timer.h"
#include "efm32_int.h"
#include "efm32_usart.h"

#include <stdint.h>
#include <stdbool.h>

#include "led.h"
#include "trace.h"
#include "radio.h"
#include "config.h"

void initClocks();
void enableTimers();
void enableInterrupts();
void basestation_main();

int main()
{
	
	// Chip errata
	CHIP_Init();
	
	// ensure core frequency has been updated
	SystemCoreClockUpdate();
	
	// start clocks
	initClocks();
	
	// init LEDs
	LED_Init();
	
	#ifdef PPS_MASTER
	
	while(1)
	{
		
		
		
	}
	
	#endif
	
	// init irqs
	enableInterrupts();
	
	// set up trace
	TRACE_Init();
	
	// GPS Init
	
	
	// enable basestation if reqd
	#ifdef BASESTATION
		
		basestation_main();
		
	#endif
	
	// Display init
	
	// display getting fix message
	
	// radio init
	RADIO_Init();
	
	// radio get id
	RADIO_GetID();
	
	// wait for gps initial fix
	
	// enable tdma
	RADIO_EnableTDMA();
	
	while(1)
	{
		
		// handle radio msgs
		RADIO_HandleMessages();
		
		// display update
		
		// gps update
		
		// sleep until irq
		
	}
	
}

void enableInterrupts()
{
	
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);
	
}

void initClocks()
{
	
	/* Starting LFXO and waiting until it is stable */
	CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

	// starting HFXO, wait till stable
	CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

	// route HFXO to CPU
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

	/* Routing the LFXO clock to the RTC */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

	// disabling the RCs
	CMU_ClockEnable(cmuSelect_HFRCO, false);
	CMU_ClockEnable(cmuSelect_LFRCO, false);

	/* Enabling clock to the interface of the low energy modules */
	CMU_ClockEnable(cmuClock_CORE, true);
	CMU_ClockEnable(cmuClock_CORELE, true);

	// enable clock to hf perfs
	CMU_ClockEnable(cmuClock_HFPER, true);

	// enable clock to GPIO
	CMU_ClockEnable(cmuClock_GPIO, true);

	// enable clock to RTC
	CMU_ClockEnable(cmuClock_RTC, true);
	RTC_Enable(true);

	// enable radio usart
	CMU_ClockEnable(cmuClock_USART2, true);
	
	// enable pc serial
	CMU_ClockEnable(cmuClock_UART1, true);
	
	// enable timers
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	
	// i2c
	CMU_ClockEnable(cmuClock_I2C0, true);
	
}

void basestation_main()
{
	
	while (!(UART1->STATUS & UART_STATUS_TXBL));
		
	while (1)
	{
		if ((UART1->STATUS & UART_STATUS_RXDATAV) && UART1->RXDATA == '*')
			break;
	}
	
	int i;
	for (i = 0; i < 500000; i++);
	
	UART1->TXDATA = '*';
	
	// wait for GPS fix
	
	
	// send GPS fix packet to pi
	
	
	RADIO_Init();
	
	uint8_t pibound_packet[32],
		meshbound_packet[32],
		pibound_position = 32,
		meshbound_position = 0;
	bool tx = false;
	
	RADIO_Enable(OFF);
	RADIO_SetMode(RX);
	RADIO_Enable(RX);
	
	while (1)
	{
		
		if (RADIO_TxBufferSize())
		{
			
			if (!RADIO_Sending())
			{
			
				RADIO_Enable(OFF);
				RADIO_SetMode(TX);
				RADIO_TxBufferFill();
				RADIO_Enable(TX);
				tx = true;
				
			}
			
		}
		else
		{
			
			if (tx)
			{
				RADIO_Enable(OFF);
				RADIO_SetMode(RX);
				RADIO_Enable(RX);
				tx = false;
			}
			
		}
		
		if (UART1->STATUS & UART_STATUS_RXDATAV)
		{
			
			meshbound_packet[meshbound_position++] = UART1->RXDATA;
			
			if (meshbound_position == 32)
			{
				// check for packets for basestation
				// (such as switch to TDMA)
				RADIO_Send(meshbound_packet);
				meshbound_position = 0;
			}
			
		}
		
		if (UART1->STATUS & UART_STATUS_TXBL)
		{
			
			if (pibound_position == 32)
			{
				
				if (RADIO_Recv(pibound_packet))
				{
					pibound_position = 0;
				}
				else
				{
					continue;
				}
				
			}
			
			UART1->TXDATA = pibound_packet[pibound_position++];
			
		}
		
	}
	
}
