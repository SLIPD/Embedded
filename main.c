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

#include "display.h"
#include "MAG3110.h"

#include <stdint.h>

#include "radio.h"
#include "led.h"
#include "trace.h"

/* variables */
DISPLAY_Message displayMessage;
uint8_t buf[192*2];
Mag_Vector_Type magReading;
char str [32];


/* prototypes */
void InitClocks();
void HandleInterrupt();
void startupLEDs();
void updateLEDs(uint8_t color);

/* functions */

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

// messy interrupt handler
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

void updateLEDs(uint8_t color)
{
	
	switch (color)
	{
	case 0:
		LED_On(RED);
		LED_Off(GREEN);
		break;
	case 1:
		LED_On(BLUE);
		LED_Off(RED);
		break;
	case 2:
		LED_On(GREEN);
		LED_Off(BLUE);
		break;
	}
	
}

void startupLEDs()
{
	
	LED_Off(RED);
	LED_Off(BLUE);
	LED_Off(GREEN);
	
	uint32_t time = RTC_CounterGet();
	while (RTC_CounterGet() < time + 32768);
	
	LED_On(RED);
	LED_On(BLUE);
	LED_On(GREEN);
	
	time = RTC_CounterGet();
	while (RTC_CounterGet() < time + 32768);
	
	LED_Off(RED);
	LED_Off(BLUE);
	LED_Off(GREEN);
	
	time = RTC_CounterGet();
	while (RTC_CounterGet() < time + 32768);
	
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
	
        // enable I2C
        CMU_ClockEnable(cmuClock_I2C0, true);
        CMU_ClockEnable(cmuClock_TIMER0, true);
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
	
	// init LEDs
	LED_Init();
        
	// show startup LEDs
	startupLEDs();
	
	// enable gpio interrupts
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
        NVIC_ClearPendingIRQ(TIMER0_IRQn);
        NVIC_EnableIRQ(TIMER0_IRQn);
	
	// init radio
	//RADIO_Init();
	//TRACE("Radio started\n");
     
        // init display
        DISPLAY_Init();
        DISPLAY_InitMessage(&displayMessage);
        
        // init MAG
        MAGInit(); // Set up magnetometer
        MAGRegReadN(OUT_X_MSB_REG, 6, buf); // Read MSB of X 
        
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	
	// set up LEDs
	uint8_t color = 0;
	uint8_t packet[RADIO_PACKET_SIZE];
	LED_Off(RED);
	LED_Off(BLUE);
	LED_Off(GREEN);
        
        MAGRegReadN(OUT_X_MSB_REG, 6, buf);
        magReading.x = buf[0]<<8 | buf[1];
        magReading.y = buf[2]<<8 | buf[3];
        magReading.z = buf[4]<<8 | buf[5];

	while(1)
        {
            if(MAGRegRead(DR_STATUS_REG) & 0x08)
            {
               // TRACE("MAG UPDATE AVAILABLE\n");
                MAGRegReadN(OUT_X_MSB_REG, 6, buf);
                magReading.x = buf[0]<<8 | buf[1];
                magReading.y = buf[2]<<8 | buf[3];
                magReading.z = buf[4]<<8 | buf[5];
                displayMessage.topLine=true;
                displayMessage.message = ("YES\n");
                DISPLAY_SetMessage(&displayMessage); 
                displayMessage.topLine=false;
                sprintf(str, "%+4i %+4i", magReading.x, magReading.y);
                displayMessage.message=(str);
                DISPLAY_SetMessage(&displayMessage); 
            }
            else
            {
               // TRACE("NO MAG UPDATE AVAILABLE\n");
                displayMessage.message = ("NO\n");
                DISPLAY_SetMessage(&displayMessage); 
            }
            DISPLAY_Update(); 
            wait(1000);
        }
}