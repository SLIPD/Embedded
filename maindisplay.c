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
#include <stdbool.h>

#include "led.h"
#include "trace.h"
#include "display.h"
#include "i2cdrv.h"

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

int main()
{
	
	// Chip errata
	CHIP_Init();
	
	// ensure core frequency has been updated
	SystemCoreClockUpdate();
	
	// start clocks
	//InitClocks();
	
	// startup trace
	//TRACE_Init();
	//TRACE("Trace started\n");
	
	// init LEDs
	
	
	// show startup LEDs
	//startupLEDs();
	
		I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	
	/* Enable LE clock and LFXO oscillator */
	CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_LE;
	CMU->OSCENCMD |= CMU_OSCENCMD_LFXOEN;
	/* Wait until LFXO ready */
	/* Note that this could be done more energy friendly with an interrupt in EM1 */
	while (!(CMU->STATUS & CMU_STATUS_LFXORDY)) ;
	
	/* Select LFXO as clock source for LFACLK */
	CMU->LFCLKSEL = (CMU->LFCLKSEL & ~_CMU_LFCLKSEL_LFA_MASK) | CMU_LFCLKSEL_LFA_LFXO;

	CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN;
	
	/* Enable GPIO clock */
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
	
	CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_I2C0;
    /* Enable signals SDA, SCL */
    I2C0->ROUTE |= I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | I2C_ROUTE_LOCATION_LOC3;
	
	GPIO_PinOutSet(gpioPortD,14);
	GPIO_PinOutSet(gpioPortD,15);
	
	GPIO_PinModeSet(gpioPortD, 15, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(gpioPortD, 14, gpioModeWiredAnd, 1);
    
    I2C_Init(I2C0, &i2cInit);
    
	//I2CDRV_Init(&i2cInit);
	
	int i, j;
	for (i = 0; i < 9; i++)
  {
    /*
     * TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
     * optimization when running at 14MHz. A bit high for standard mode devices,
     * but DVK only has fast mode devices. Need however to add some time
     * measurement in order to not be dependable on frequency and code executed.
     */
    GPIO_PinOutClear(gpioPortD, 15);
    GPIO_PinOutSet(gpioPortD, 15);
  }
	
	
  LED_Init();
	LED_Off(BLUE);
	LED_Off(RED);
	LED_Off(GREEN);
	
	for (j = 0; j < 6; j++)
	{
		LED_Toggle(GREEN);
		for (i = 0; i < 500000; i++);
	}
	
	LED_On(RED);
	
	I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
	
	/*
  seq.addr = 0x007C;
  seq.flags = I2C_FLAG_WRITE_WRITE;
  
  uint8_t control = 0x00;
  uint8_t data = 0x38;
  
  seq.buf[0].data = &control;
  seq.buf[0].len = 1;
  seq.buf[1].data = &data;
  seq.buf[1].len = 1;
 
  ret = I2CDRV_Transfer(&seq);
  */
  
  GPIO_PinModeSet(gpioPortE,9,gpioModePushPull,0);
  GPIO_PinOutClear(gpioPortE,9);
  for (i = 0; i < 500000; i++);
  LED_On(RED);
  GPIO_PinOutSet(gpioPortE,9);
  for (i = 0; i < 500000; i++);
  LED_Off(RED);
  GPIO_PinOutClear(gpioPortE,9);
  
  uint8_t regid[1];
  uint8_t data[1];

  seq.addr = 0x007C; // 38
  seq.flags = I2C_FLAG_WRITE_WRITE;
  /* Select register to be read */
  
  regid[0] = 0x00; // 2a
  data[0] = 0x38;
  
  seq.buf[0].data = regid;
  seq.buf[0].len = 1;
  seq.buf[1].data = data;
  seq.buf[1].len = 1;
	
	//int i;
	
	ret = I2CDRV_Transfer(&seq);
  
  LED_Off(RED);
  
  if (ret == i2cTransferDone)
  {
	
		LED_On(GREEN);
	
	}
	else if (ret == i2cTransferNack)
	{
		
		LED_On(BLUE);
		
	}

	while(1);

}