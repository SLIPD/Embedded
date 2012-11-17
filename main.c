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
#include "efm32_int.h"

#include "display.h"
#include "MAG3110.h"
#include "MMA845XQ.h"

#include <stdint.h>
#include <math.h>


#include "radio.h"
#include "led.h"
#include "trace.h"

/* variables */
DISPLAY_Message displayMessage;
uint8_t buf[192*2];
Mag_Vector_Type magReading;
Accel_Vector_Type accelReading, accelBase;
int16_t angleX, angleY;

char t_str [192*2];

// Roll, pitch and yaw angles computed by 1ecompass
int16_t iPhi, iThe, iPsi;

// Magnetic fiedl readings corrected for hard iron effects and PCB orientation
int16_t iBfx, iBfy, iBfz;

// Hard iron estimate
int16_t iVx, iVy, iVz;

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

void calc_xy_angles(float xA, float yA, float zA)
{
    float x, y, z;
    uint16_t x2, y2, z2;
   
    // Work out squares
    x = xA - (float)accelBase.x;
    y = yA - (float)accelBase.y;
    z = zA - (float)accelBase.z;
    
    accelBase.x = (uint16_t) xA;
    accelBase.y = (uint16_t) yA;
    accelBase.z = (uint16_t) zA;
   
    x2 = (uint16_t) (x*x);
    y2 = (uint16_t) (y*y);
    z2 = (uint16_t) (z*z);
    
    angleX = x / sqrt(y2 + z2);
    angleY = y / sqrt(x2 + z2);
    
    sprintf(t_str, "x %d, y %d\n", angleX, angleY);
    TRACE(t_str);
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
	
    uint8_t verbose = 1;
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
        uint8_t a = MAGRegRead(WHO_AM_I);
        sprintf(t_str, "MAG3110 WHO AM I: 0x%2x\n", a);
        TRACE(t_str);
        
        // init MMA
        MMAInit(); // Set up accelerometer
        MMARegReadN(OUT_X_MSB_REG, 6, buf); // Read MSB of X 
        accelBase.x = buf[0]<<8 | buf[1];
        accelBase.y = buf[2]<<8 | buf[3];
        accelBase.z = buf[4]<<8 | buf[5];
        a = MMARegRead(WHO_AM_I_REG);
        sprintf(t_str, "MMA WHO AM I: 0x%2x\n", a);
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
        
        int count = 0;
        int32_t xC, yC, zC;
        
        TRACE("Collecting values!\n");
        while( count < 100)
        {
            if(MMARegRead(DR_STATUS_REG) & ZYXDR_MASK)
            {
               MMARegReadN(OUT_X_MSB_REG, 6, buf);
               accelReading.x = buf[0]<<8 | buf[1];
               accelReading.y = buf[2]<<8 | buf[3];
               accelReading.z = buf[4]<<8 | buf[5];
               xC += accelReading.x;
               yC += accelReading.y;
               zC += accelReading.z;                       
               sprintf(t_str, "Count x %d, y %d, z %d\n",  xC, yC, zC);
               TRACE(t_str);
               count++;
           }
        }
        
        calc_xy_angles((float)xC / 200, (float)yC / 200, (float)zC / 200);
        
        sprintf(t_str, "x %d, y %d\n", angleX, angleY);
        TRACE(t_str);
        
	while(1)
        {
            // If there is new ZYX data available
            if(MMARegRead(DR_STATUS_REG) & ZYXDR_MASK)
            {
               MMARegReadN(OUT_X_MSB_REG, 6, buf);
               accelReading.x = buf[0]<<8 | buf[1];
               accelReading.y = buf[2]<<8 | buf[3];
               accelReading.z = buf[4]<<8 | buf[5];
               
               calc_xy_angles(accelReading.x, accelReading.y, accelReading.z);
               
                wait(500);
            }
            else
            {
               //TRACE("NO MAG UPDATE AVAILABLE\n");
        
            }
            wait(1000);
        }
}