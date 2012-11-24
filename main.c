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
#include "MAG3110.h"
#include "MMA845XQ.h"

#include "gps.h"

// Global variables
DISPLAY_Message         displayMessageTop;
DISPLAY_Message         displayMessageBottom;
Mag_Vector_Type         magReading;
Accel_Vector_Type       accelReading;
uint8_t buf[6];
char str[32];

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
        displayMessageTop.message=("  ^   ___   ^   ");
        DISPLAY_SetMessage(&displayMessageTop); 
        
        displayMessageBottom.topLine = false;
        displayMessageBottom.scroll = false;
        displayMessageBottom.message=("      __/      ");
        DISPLAY_SetMessage(&displayMessageBottom);  
        
	// display getting fix message
        
        // magnetometer init
        MAGInit(); 
        MAGRegReadN(MAG_OUT_X_MSB_REG, 6, buf); // Read MSB of X 
        magReading.x = buf[0]<<8 | buf[1];
        magReading.y = buf[2]<<8 | buf[3];
        magReading.z = buf[4]<<8 | buf[5];
        
        uint8_t a = MAGRegRead(MAG_SYSMOD);
        sprintf(str, "SYSMOD = 0x%2.2x, %d\n", a);
        TRACE(str);

        // accelerometer init
        MMAInit();
        MMARegReadN(OUT_X_MSB_REG, 6, buf); // Read MSB of X 
        accelReading.x = ((int16_t) (buf[0]<<8 | buf[1])) >> 0x2;
        accelReading.y = ((int16_t) (buf[2]<<8 | buf[3])) >> 0x2;
        accelReading.z = ((int16_t) (buf[4]<<8 | buf[5])) >> 0x2;
        
	// radio init
	
	// radio get id
	
	// wait for gps initial fix
	
	// enable tdma
	
	while(1)
	{
                // handle radio msgs
                
		// display update
                DISPLAY_Update();
		// gps update
                // If there is new ZYX data available, read
                if(MAGRegRead(MAG_DR_STATUS_REG) & MAG_ZYXDR_MASK)
                {
                    MAGRegReadN(MAG_OUT_X_MSB_REG, 6, buf);
                    magReading.x = buf[0]<<8 | buf[1];
                    magReading.y = buf[2]<<8 | buf[3];
                    magReading.z = buf[4]<<8 | buf[5];
                    
                    sprintf(str, "x %d, y %d, z %d\n", magReading.x, magReading.y, magReading.z);
                    TRACE(str);
                }
		// sleep until irq
                for(int i = 0; i < 1000000; i++);
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
