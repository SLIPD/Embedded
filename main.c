/*

Main file for SLIP D embedded software

*/

/* includes */
#include "efm32.h"

#include "efm32.h"
#include "efm32_chip.h"
#include "efm32_emu.h"
#include "efm32_gpio.h"
#include "efm32_i2c.h"
#include "efm32_usart.h"
#include "efm32_rtc.h"
#include "efm32_cmu.h"
#include "efm32_adc.h"
#include "efm32_timer.h"

#include <stdint.h>

#include "led.h"
#include "trace.h"
#include "radio.h"
#include "display.h"
#include "i2cdrv.h"

/* variables */


/* prototypes */
void InitClocks();
void startupLEDs();
void wait(uint32_t ms);

/* functions */
void GPIO_EVEN_IRQHandler(void) 
{
	HandleInterrupt();
}
void GPIO_ODD_IRQHandler(void)
{
	HandleInterrupt();
}

void HandleInterrupt()
{
	TRACE("INTERRUPT RECEIVED\n");
	RADIO_Interrupt();
}

void wait(uint32_t ms)
{
	
	uint32_t time, 
		clockFreq = CMU_ClockFreqGet(cmuClock_RTC);
	
	while (ms > 0)
	{
		
		time = RTC_CounterGet();
		
		if (16777215 - time < ((double)ms / 1000.0) * clockFreq)
		{
			ms -= (uint32_t)(1000.0 * ((16777215 - time) / (double)clockFreq));
			while (RTC_CounterGet() > time);
		}
		else
		{
			while (RTC_CounterGet() < time + ((double)ms / 1000.0) * clockFreq);
			break;
		}
		
	}
	
}

void startupLEDs()
{
	
	LED_Off(RED);
	LED_Off(BLUE);
	LED_Off(GREEN);
	
	wait(1000);
	
	LED_On(RED);
	LED_On(BLUE);
	LED_On(GREEN);
	
	wait(1000);
	
	LED_Off(RED);
	LED_Off(BLUE);
	LED_Off(GREEN);
	
	wait(1000);
	
}

void InitClocks()
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
	
	// enable clock to USARTs
	CMU_ClockEnable(cmuClock_UART1, true);
	CMU_ClockEnable(cmuClock_USART2, true);
	
	CMU_ClockEnable(cmuClock_I2C0, true);
	
}

void TIMER0_IRQHandler(void)
{ 
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);
  
  /* Toggle LED ON/OFF */
  LED_Toggle(RED);
}

int main()
{
	
	// Chip errata
	CHIP_Init();
	
	// ensure core frequency has been updated
	SystemCoreClockUpdate();
	
	// start clocks
	InitClocks();
	
	// startup trace
	TRACE_Init();
	TRACE("Trace started\n");
	
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	
	// init LEDs
	LED_Init();
	
	// show startup LEDs
	startupLEDs();
	
	#ifdef RECEIVER
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_I2C0;
	I2C0->ROUTE |= I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC3;
	
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	
	I2C0->CLKDIV=1;
	
	I2CDRV_Init(&i2cInit);
	
	DISPLAY_Init();
	#endif
	
	// radio
	RADIO_Init();
	
	#ifdef SENDER
	uint8_t buffer[32], pos = 0;
	
	while (!(UART1->STATUS & USART_STATUS_TXBL));
		
	UART1->TXDATA = 0xFF;
	
	while (1)
	{
		
		if (pos == 32)
		{
			
			// send over radio
			while (!RADIO_Ready());
			RADIO_Transmit(buffer);
			
			pos = 0;
			
			while (!(UART1->STATUS & USART_STATUS_TXBL));
		
			UART1->TXDATA = 0xFF;
			
		}
		
		while (!(UART1->STATUS & USART_STATUS_RXDATAV));
		
		buffer[pos] = UART1->RXDATA;
		pos++;
		
	}
	
	#endif
	
	while (1);
	
}