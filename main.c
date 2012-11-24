#include "efm32.h"

#include "efm32_chip.h"
#include "efm32_rtc.h"
#include "efm32_gpio.h"
#include "efm32_cmu.h"
#include "efm32_timer.h"
#include "efm32_int.h"
#include "efm32_i2c.h"

#include <stdint.h>
#include <stdbool.h>

#include "led.h"
#include "trace.h"

#include "i2cdrv.h"
#include "display.h"

#include "gps.h"

// Global variables
DISPLAY_Message displayMessageTop;
DISPLAY_Message displayMessageBottom;
char t_str [32]; // top line
char b_str [32]; // bottom line

// Prototypes
void initClocks();
void enableTimers();
void enableInterrupts();

int main()
{
	
	// Chip errata
	CHIP_Init();
	
	// ensure core frequency has been updated
	SystemCoreClockUpdate();
	
	// start clocks
	initClocks();
             
        // I2C setup
        I2C_Setup();
        
        TRACE_Init();
        TRACE("Trace started!\n");
        
	// init LEDs
	LED_Init();

	// set up trace
	TRACE_Init();
	
	// GPS Init
	
	// Display init
        DISPLAY_Init();
        DISPLAY_InitMessage(&displayMessageTop);
        DISPLAY_InitMessage(&displayMessageBottom);
        
        displayMessageTop.scroll = false;
        displayMessageTop.message=("i_am_the_top");
        DISPLAY_SetMessage(&displayMessageTop);  
        
        displayMessageBottom.scroll = false;
        displayMessageBottom.topLine=false;
        displayMessageBottom.message=("1..2..3..4..5....");
        DISPLAY_SetMessage(&displayMessageBottom); 
        
	// display getting fix message
	
	// radio init
	
	// radio get id
	
	// wait for gps initial fix
	
	// enable tdma
	
	while(1)
	{
                // handle radio msgs

		// display update

		// gps update

		// sleep until irq
	}
	
}

void enableInterrupts()
{
	
	NVIC_EnableIRQ(USART2_TX_IRQn);
	NVIC_EnableIRQ(USART2_RX_IRQn);
	
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
	
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
        
        // !!!
        // FOR GPS, ENABLE CLOCK FOR LEUART AND LOW ENERGY MODULES
	// !!!
//        CMU_ClockEnable(cmuClock_LEUART1, true);
        
	// enable pc serial
	CMU_ClockEnable(cmuClock_UART1, true);
	
	// enable timers
	CMU_ClockEnable(cmuClock_TIMER0, true);
	CMU_ClockEnable(cmuClock_TIMER1, true);
	
	// i2c
	CMU_ClockEnable(cmuClock_I2C0, true);
	
}
