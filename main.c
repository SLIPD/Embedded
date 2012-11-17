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
Accel_Vector_Type accelReading;
//char t_str [32];
char t_str [192*2];
char b_str [32];
float heading, headingDegrees, declinationAngle;
float rotMatrix[3][3];
int16_t xEl[500], yEl[500], zEl[500];
int16_t xOff, yOff, zOff;
float g = 9.81;

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

void quickSort(int16_t x[500],int first,int last)
{
    int pivot, j, temp, i;

     if(first < last)
     {
         pivot = first;
         i = first;
         j = last;

         while(i < j)
         {
             while(x[i] <= x[pivot] && i < last)
                 i++;
             while(x[j] > x[pivot])
                 j--;
             if(i < j){
                 temp = x[i];
                  x[i] = x[j];
                  x[j] = temp;
             }
         }

         temp = x[pivot];
         x[pivot] = x[j];
         x[j] = temp;
         quickSort(x, first, j-1);
         quickSort(x, j+1, last);
    }
}
    
void sortEllipsis()
{
    quickSort(xEl, 0, 499);
    quickSort(yEl, 0, 499);
    quickSort(zEl, 0, 499);
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

void initMatrix()
{
    int i, j;
    for (i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++) 
            rotMatrix[i][j] = 0;
    }
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
	
	if(verbose) TRACE("A\n");
	
	// enable gpio interrupts
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
        NVIC_ClearPendingIRQ(TIMER0_IRQn);
        NVIC_EnableIRQ(TIMER0_IRQn);
	initMatrix();
        
	if(verbose) TRACE("B\n");
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
        
	if(verbose) TRACE("C\n");
        // init MAG
        MAGInit(); // Set up magnetometer
        
	if(verbose) TRACE("D\n");
        MAGRegReadN(OUT_X_MSB_REG, 6, buf); // Read MSB of X 
        uint8_t a = MAGRegRead(WHO_AM_I);
        sprintf(t_str, "MAG3110 WHO AM I: 0x%2x\n", a);
        TRACE(t_str);
        
        // init MMA
        MMAInit(); // Set up accelerometer
        MMARegReadN(OUT_X_MSB_REG, 6, buf); // Read MSB of X 
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
        
        declinationAngle = 0 / 1000;
        
        int count = 0;
        
        INT_Enable();
        
        if(0)
        //while(count<500)
        {
            if(MAGRegRead(DR_STATUS_REG) & ZYXDR_MASK)
            {
                MAGRegReadN(OUT_X_MSB_REG, 6, buf);
                magReading.x = buf[0]<<8 | buf[1];
                magReading.y = buf[2]<<8 | buf[3];
                magReading.z = buf[4]<<8 | buf[5];
                xEl[count] = magReading.x;
                yEl[count] = magReading.y;
                zEl[count] = magReading.z;                       
                sprintf(t_str, "Ell x %d, y %d, z %d\n",   xEl[count],  yEl[count], zEl[count]);
                TRACE(t_str);
                count++;
            }
                
        }
        
//        sortEllipsis();
//        
//        TRACE("Sorted!\n");
//        
//        xOff = (xEl[0] + xEl[499])/2;
//        yOff = (yEl[0] + yEl[499])/2;
//        zOff = (zEl[0] + zEl[499])/2;
//        
//        MAGRegWrite(OFF_X_MSB , xOff >> 8); 
//        sprintf(t_str, "OFF_X_MSB 0x%2.2x %d\n", (xOff >> 8) & 0xFF, (xOff >> 8) & 0xFF);
//        TRACE(t_str);
//        MAGRegWrite(OFF_X_LSB , xOff & 0xFF); 
//        sprintf(t_str, "OFF_X_LSB 0x%2.2x %d\n",  xOff & 0xFF,  xOff & 0xFF);
//        TRACE(t_str);
//        sprintf(t_str, "OFF_X 0x%4.4x %d\n\n",  xOff & 0xFFFF, xOff & 0xFFFF);
//        TRACE(t_str);
//        
//        MAGRegWrite(OFF_Y_MSB , yOff >> 8); 
//        sprintf(t_str, "OFF_Y_MSB 0x%2.2x %d\n", (yOff >> 8) & 0xFF, (yOff >> 8) & 0xFF);
//        TRACE(t_str);
//        MAGRegWrite(OFF_Y_LSB , yOff  & 0xFF); 
//        sprintf(t_str, "OFF_Y_LSB 0x%2.2x %d\n",  yOff & 0xFF,  yOff & 0xFF);
//        TRACE(t_str);
//        sprintf(t_str, "OFF_Y 0x%4.4x %d\n\n",  yOff & 0xFFFF, yOff & 0xFFFF);
//        TRACE(t_str);
//        
//        MAGRegWrite(OFF_Z_MSB , zOff >> 8); 
//        sprintf(t_str, "OFF_Z_MSB 0x%2.2x %d\n",  (zOff >> 8) & 0xFF,  (zOff >> 8) & 0xFF);
//        TRACE(t_str);
//        MAGRegWrite(OFF_Z_LSB , zOff  & 0xFF); 
//        sprintf(t_str, "OFF_Z_LSB 0x%2.2x %d\n", zOff & 0xFF, zOff & 0xFF);
//        TRACE(t_str);
//        sprintf(t_str, "OFF_Z 0x%4.4x %d\n\n",  zOff & 0xFFFF, zOff & 0xFFFF);
//        TRACE(t_str);
        
	while(1)
        {
            // If there is new ZYX data available
            if(0)
            //if(MAGRegRead(DR_STATUS_REG) & ZYXDR_MASK)
            {
               // TRACE("MAG UPDATE AVAILABLE\n");
                MAGRegReadN(OUT_X_MSB_REG, 6, buf);
                magReading.x = buf[0]<<8 | buf[1];
                magReading.y = buf[2]<<8 | buf[3];
                magReading.z = buf[4]<<8 | buf[5];
                if(1)
                {
                        TRACE("Mag ");
                        sprintf(t_str, "x %d, y %d, z %d\n", magReading.x, magReading.y, magReading.z);
                        TRACE(t_str);  
                }       
                
                MMARegReadN(OUT_X_MSB_REG, 6, buf);
                accelReading.x = buf[0]<<8 | buf[1];
                accelReading.y = buf[2]<<8 | buf[3];
                accelReading.z = buf[4]<<8 | buf[5];
                if(0)
                { 
                        TRACE("Accel ");
                        sprintf(t_str, "x %d, y %d, z %d\n", accelReading.x, accelReading.y, accelReading.z);
                        TRACE(t_str);  
                }
                
                heading = 0; 
                
                /*
                if ((magReading.x == 0)&&(magReading.y < 0))   
                    heading = PI/2.0;  
                else if ((magReading.x == 0)&&(magReading.y > 0))               
                    heading=3.0*PI/2.0;  
                else if (magReading.x < 0)                                 
                    heading = PI - atan(magReading.y/magReading.x);  
                else if ((magReading.x > 0)&&(magReading.y < 0))           
                    heading = -atan(magReading.y/magReading.x);  
                else if ((magReading.x > 0)&&(magReading.y > 0))           
                    heading = 2.0*PI - atan(magReading.y/magReading.x);
                else
                    TRACE("FAILED\n");
                */
                
                
                
                
                if(magReading.y > 0)                            heading = 90 - (atan2(magReading.y,magReading.x)) * (180 / PI);
                else if (magReading.y < 0)                      heading = 270 - (atan2(magReading.y,magReading.x)) * (180 / PI);
                else if (magReading.y = 0 && magReading.x < 0)  heading = 180.0;
                else if (magReading.y = 0 && magReading.x > 0)  heading = 0.0;
                else
                {
                    TRACE("No constraint satisfied\n");
                }
                
                if(heading < 0)        heading+= 360;
                if(heading > 360)      heading-= 360;
                 
                
                
                
                
                /*
                heading = atan2(magReading.y, magReading.x);
                heading -= declinationAngle;
                if(heading < 0)
                    heading+= 2*PI;
                
                if(heading > 2*PI)
                    heading -=2*PI;
               
                headingDegrees = heading * (180/PI);
                
                sprintf(t_str, "heading %.2f, deg %.2f\n", heading, headingDegrees);
                */
                
                
                
                if(heading < 0)
                    heading+= 2*PI;
                
                if(heading > 2*PI)
                    heading -=2*PI;
               
                headingDegrees = heading * (180/PI);
                sprintf(t_str, "heading %.2f, deg %.2f\n", heading, headingDegrees);
                TRACE(t_str);
                
                
                
                
                /*  
                displayMessage.topLine=true;
                sprintf(t_str, "y %.2f x %.2f", magReading.y, magReading.x);
                displayMessage.message=(t_str);
                DISPLAY_SetMessage(&displayMessage); 
                
                displayMessage.topLine=false;
                sprintf(b_str, "%.2f %.2f", heading, headingDegrees);
                displayMessage.message=(b_str);
                DISPLAY_SetMessage(&displayMessage); 
                */
                
                wait(500);
            }
            else
            {
               //TRACE("NO MAG UPDATE AVAILABLE\n");
                  
                MMARegReadN(OUT_X_MSB_REG, 6, buf);
                accelReading.x = buf[0]<<8 | buf[1];
                accelReading.y = buf[2]<<8 | buf[3];
                accelReading.z = buf[4]<<8 | buf[5];
                if(1)
                { 
                        TRACE("Accel ");
                        sprintf(t_str, "x %d, y %d, z %d\n", accelReading.x, accelReading.y, accelReading.z);
                        TRACE(t_str);  
                }
               // displayMessage.message = ("NO\n");
                //DISPLAY_SetMessage(&displayMessage); 
            }
            //DISPLAY_Update(); 
            wait(1000);
        }
}