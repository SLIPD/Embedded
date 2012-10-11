#include "gps.h"

#include "stdint.h"
#include "string.h"

#include "efm32.h"

#include "efm32_gpio.h"
#include "efm32_leuart.h"
#include "efm32_rtc.h"
#include "efm32_cmu.h"

#include "trace.h"
#include "radio.h"

/* variables */
uint32_t last_mode_change = 0;

/* prototypes */
void switchMode();

/* functions */
void GPS_Init()
{
	
	LEUART_Init_TypeDef leuart1Init =
	{
		.enable   = leuartEnable,       /* Activate data reception on LEUn_TX pin. */
		.refFreq  = CMU_ClockFreqGet(cmuClock_LEUART1),                    /* Inherit the clock frequenzy from the LEUART clock source */
		.baudrate = 4800,                 /* Baudrate = 9600 bps */
		.databits = leuartDatabits8,      /* Each LEUART frame containes 8 databits */
		.parity   = leuartNoParity,       /* No parity bits in use */
		.stopbits = leuartStopbits2,      /* Setting the number of stop bits in a frame to 2 bitperiods */
	};
	
  LEUART_Reset(LEUART1);
  LEUART_Init(LEUART1, &leuart1Init);
	
	// config uart
	//LEUART1->CLKDIV = 256 * ((CMU_ClockFreqGet(cmuClock_LEUART1)) / (4800 * 2) - 1);
	
	//LEUART1->CMD = LEUART_CMD_TXEN | LEUART_CMD_RXEN;
	
	//GPIO->P[0].DOUT |= (1 << 5);
	GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_PUSHPULL;
	GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE6_MASK) | GPIO_P_MODEL_MODE6_INPUT;
	
	LEUART1->ROUTE = LEUART_ROUTE_LOCATION_LOC1
          | LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN;
	
	// config wakeup pin (goes into hibernate mode)
	//GPIO->P[4].DOUT &= ~(1 << 12);
	GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE12_MASK) | GPIO_P_MODEH_MODE12_INPUT;
	
	// config on off pin
	GPIO->P[4].DOUT &= ~(1 << 13);
	GPIO->P[4].MODEH = (GPIO->P[4].MODEH & ~_GPIO_P_MODEH_MODE13_MASK) | GPIO_P_MODEH_MODE13_PUSHPULL;
	
	//LEUART1->CMD = LEUART_CMD_CLEARRX | LEUART_CMD_CLEARTX;
	
	RTC_CounterReset();
	while (!(GPIO->P[4].DIN & (1 << 12)))
	{
		switchMode();
	}
	
	
	/*
	char* msg = "$PSRF105,1*3E\r\n";
	int i, len = strlen(msg);
	
	TRACE("Enabling debug...");
	
	for (i = 0; i < len; i++)
	{
		while(!(LEUART1->STATUS & LEUART_STATUS_TXBL));
		LEUART1->TXDATA = *msg++;
	}
	while(!(LEUART1->STATUS & LEUART_STATUS_TXC));
	
	TRACE("done\n");
	
	//LEUART1->CLKDIV = 256 * ((CMU_ClockFreqGet(cmuClock_LEUART1)) / (9600 * 16) - 1);
	
	TRACE("Starting GPS output:\n");
	
	// read stuff from gps chip
	char byte[2];
	byte[1] = 0;
	i = 0;
	
	while (1)
	{
		
		if (LEUART1->STATUS & LEUART_STATUS_RXDATAV)
		{
			byte[0] = LEUART1->RXDATA;
			TRACE(byte);
		}
		
	}*/
	
}

void switchMode()
{
	
	// wait at least 1 second since last change
	while (RTC_CounterGet() < last_mode_change + CMU_ClockFreqGet(cmuClock_RTC));
	
	// disable interrupts as this is time sensitive
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	
	GPIO->P[4].DOUT |= (1 << 13);
	
	uint32_t time = RTC_CounterGet(),
		wait = CMU_ClockFreqGet(cmuClock_RTC) / 10;
	
	while (RTC_CounterGet() < time + wait);
	
	GPIO->P[4].DOUT &= ~(1 << 13);
	
	time = RTC_CounterGet();
	wait = CMU_ClockFreqGet(cmuClock_RTC);
	while (RTC_CounterGet() < time + wait);
	
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	
	last_mode_change = RTC_CounterGet();
	
}

void GPS_Read(uint8_t packet[])
{
	
	uint8_t i = 0;
	uint8_t msg[2];
	msg[1] = 0;
	
	do
	{
		
		if (LEUART1->STATUS & LEUART_STATUS_RXDATAV)
		{
			msg[0] = LEUART1->RXDATA;
			TRACE(msg);
			
			i++;
			packet[i] = msg[0];
		}
		
		//TRACE("STUCK");
		
	} while (packet[i] != '\n' && i < RADIO_PACKET_SIZE - 2);
	
	//TRACE("======================OK========================");
	
	packet[i+1] = 0;
	
}