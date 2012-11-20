/*

Main file for SLIP D embedded software

*/

/* includes */
/* Standard libraries */
#include <stdint.h>
#include <math.h>

/* EFM libraries */
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
#include "efm32_int.h"

/* SLIP D Embedded libraries*/
#include "radio.h"
#include "led.h"
#include "trace.h"
#include "display.h"

/* Custom libraries */
#include "MAG3110.h"
#include "MMA845XQ.h"
#include "eCompass.h"


/* variables */
DISPLAY_Message displayMessage;
uint8_t buf[192*2];
Mag_Vector_Type         magReading;
Accel_Vector_Type       accelReading;

char t_str [192*2];

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
        
        // init display AND I2C
//        DISPLAY_Init();
//        DISPLAY_InitMessage(&displayMessage);
        
	// init reset pin
	GPIO_PinModeSet(gpioPortD,10,gpioModePushPull,1);
	GPIO_PinModeSet(gpioPortD, 15, gpioModeWiredAnd, 1);
	GPIO_PinModeSet(gpioPortD, 14, gpioModeWiredAnd, 1);

	// init i2c
	I2C0->ROUTE |= I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC3;

	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

	I2C_Init(I2C0, &i2cInit);
        
        // init MAG
        MAGInit(); // Set up magnetometer
        MAGRegReadN(OUT_X_MSB_REG, 6, buf); // Read MSB of X 
        magReading.x = buf[0]<<8 | buf[1];
        magReading.y = buf[2]<<8 | buf[3];
        magReading.z = buf[4]<<8 | buf[5];
        uint8_t id = MAGRegRead(WHO_AM_I);
        sprintf(t_str, "MAG3110 WHO AM I: 0x%2x\n", id);
        TRACE(t_str);
        
        // init MMA
        MMAInit(); // Set up accelerometer
        MMARegReadN(OUT_X_MSB_REG, 6, buf); // Read MSB of X 
        accelReading.x = ((int16_t) (buf[0]<<8 | buf[1])) >> 0x2;
        accelReading.y = ((int16_t) (buf[2]<<8 | buf[3])) >> 0x2;
        accelReading.z = ((int16_t) (buf[4]<<8 | buf[5])) >> 0x2;
        id = MMARegRead(WHO_AM_I_REG);
        sprintf(t_str, "MMA WHO AM I: 0x%2x\n", id);
        TRACE(t_str);
        
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	
	// set up LEDs
	uint8_t color = 0;
	uint8_t packet[RADIO_PACKET_SIZE];
	LED_Off(RED);
	LED_Off(BLUE);
	LED_Off(GREEN);
        
        INT_Enable();
        
        TRACE("eCompassInit() called\n!");
        eCompassInit();
        
	while(1)
        {
            // If there is new ZYX data available, read
            if(MAGRegRead(DR_STATUS_REG) & ZYXDR_MASK)
            {
                MAGRegReadN(OUT_X_MSB_REG, 6, buf);
                magReading.x = buf[0]<<8 | buf[1];
                magReading.y = buf[2]<<8 | buf[3];
                magReading.z = buf[4]<<8 | buf[5];
            }
            
            if(MMARegRead(DR_STATUS_REG) & ZYXDR_MASK)
            {
                MMARegReadN(OUT_X_MSB_REG, 6, buf);
                accelReading.x = ((int16_t) (buf[0]<<8 | buf[1])) >> 0x2;
                accelReading.y = ((int16_t) (buf[2]<<8 | buf[3])) >> 0x2;
                accelReading.z = ((int16_t) (buf[4]<<8 | buf[5])) >> 0x2;
            }
           
            int16_t heading = ieCompass(magReading.x, magReading.y, magReading.z, accelReading.x, accelReading.y, accelReading.z);

            sprintf(t_str,"heading 0x%4.4x %d degrees\n", heading, heading/100);
            TRACE(t_str);
            
            wait(2000);
       }
}