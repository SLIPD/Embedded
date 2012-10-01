/*

Main file for SLIP D embedded software

*/

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

int main()
{
	
	// Chip errata
	CHIP_Init();
	
	// enable RTC
	/* Starting LFXO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  /* Routing the LFXO clock to the RTC */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);

  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);
	
	// enable clock to GPIO
	CMU_ClockEnable(cmuClock_GPIO, true);
	
	// enable clock to RTC
	CMU_ClockEnable(cmuClock_RTC, true);
	
	// enable RTC
	RTC_Enable(true);
	
	// ensure core frequency has been updated
	SystemCoreClockUpdate();
	
	// set pins as output
	GPIO_PinModeSet(gpioPortA, 0, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortA, 1, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortA, 3, gpioModeWiredAnd, 1);
	
	// turn on leds
	//GPIO->P[0].DOUT &= ~(1 << 0);
	//GPIO->P[0].DOUT &= ~(1 << 1);
	//GPIO->P[0].DOUT &= ~(1 << 3);
	
	uint32_t current_time = RTC_CounterGet();
	uint32_t red_time = current_time, blue_time = current_time, green_time = current_time;
	
	// loop forever
	while(1)
	{
		
		current_time = RTC_CounterGet();
		
		if ((current_time - red_time) > 32768)
		{
			red_time = current_time;
			
			GPIO->P[0].DOUT ^= (1 << 0);
		}
		
		if ((current_time - green_time) > 16384)
		{
			green_time = current_time;
			
			GPIO->P[0].DOUT ^= (1 << 1);
		}
		
		if ((current_time - blue_time) > 8192)
		{
			blue_time = current_time;
			
			GPIO->P[0].DOUT ^= (1 << 3);
		}
		
	}
	
}