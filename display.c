#include "display.h"

#include "efm32.h"

#include "efm32_cmu.h"
#include "efm32_rtc.h"

#include "led.h"

#include "efm32_i2c.h"
#include "i2cdrv.h"
#include "trace.h"

/* prototypes */
void read(uint8_t instruction, uint8_t *data, uint8_t len);
void write(uint8_t instruction, uint8_t *data, uint8_t len);

void panic()
{
	LED_On(RED);
	LED_On(BLUE);
	LED_On(GREEN);
	while(1);
}

void wait_i2c(uint32_t ms)
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

void transfer(uint8_t control, uint8_t data)
{
	
	I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = 0x7C;      // Parameter Address
  seq.flags = I2C_FLAG_WRITE_WRITE;     // Indicate combined write / read seq 
  /* Select register to be read */
  // Format: Reg ID, Value
  // Length must be specified
  seq.buf[0].data = &control;
  seq.buf[0].len = 1;
  seq.buf[1].data = &data;
  seq.buf[1].len = 1;
 
	do
	{
		
		TRACE("\nSTART TRANSFER\n");
		wait_i2c(10);
		
		ret = I2CDRV_Transfer(&seq);
		
		if (ret != i2cTransferDone)
		{
			
			TRACE("ERROR: ");
			
			switch (ret)
			{
				
				case i2cTransferInProgress:
					TRACE("in progress\n");
					break;
				case i2cTransferNack:
					TRACE("nack\n");
					break;
				case i2cTransferBusErr:
					TRACE("bus error\n");
					break;
				case i2cTransferArbLost:
					TRACE("list arbitration\n");
					break;
				case i2cTransferUsageFault:
					TRACE("usage fault\n");
					break;
				case i2cTransferSwFault:
					TRACE("sw fault\n");
					break;
				default:
					TRACE("unknown\n");
					break;
				
			}
			
		}
	
	} while (ret != i2cTransferDone);

	TRACE("Transfer OK\n");
}

int showCursor()
{
  
  /*
  TRACE("Enabling cursor\n");
  
  wait_i2c(100);
	transfer(0x00,0x0F);
	
	TRACE("WRITING HELLO WORLD\n");
	
	transfer(0x40,0x48);
	transfer(0x40,0x45);
	transfer(0x40,0x4C);
	transfer(0x40,0x4C);
	transfer(0x40,0x4F);
	
	transfer(0x40,0x20);
	
	transfer(0x40,0x57);
	transfer(0x40,0x4F);
	transfer(0x40,0x52);
	transfer(0x40,0x4C);
	transfer(0x40,0x44);
	
	transfer(0x40,0x20);
	
	transfer(0x40,0x48);
	transfer(0x40,0x45);
	transfer(0x40,0x4C);
	transfer(0x40,0x4C);
	transfer(0x40,0x4F);
	
	transfer(0x40,0x20);
	
	transfer(0x40,0x57);
	transfer(0x40,0x4F);
	transfer(0x40,0x52);
	transfer(0x40,0x4C);
	transfer(0x40,0x44);
	*/
  
  LED_On(GREEN);

  return(0);
	
}

void write(uint8_t instruction, uint8_t *data, uint8_t len)
{
	
}

void DISPLAY_Init()
{
	
	transfer(0x00,0x38);
	//wait_i2c(100);
	transfer(0x00,0x39);
	//wait_i2c(100);
	transfer(0x00,0x14);
	//wait_i2c(100);
	transfer(0x00,0x74);
	//wait_i2c(100);
	transfer(0x00,0x54);
	//wait_i2c(100);
	transfer(0x00,0x6F);
	//wait_i2c(100);
	transfer(0x00,0x0C);
	//wait_i2c(100);
	transfer(0x00,0x01);
	
	transfer(0x40,0x48);
	transfer(0x40,0x45);
	transfer(0x40,0x4C);
	transfer(0x40,0x4C);
	transfer(0x40,0x4F);
	
	transfer(0x40,0x20);
	
	transfer(0x40,0x57);
	transfer(0x40,0x4F);
	transfer(0x40,0x52);
	transfer(0x40,0x4C);
	transfer(0x40,0x44);
	
}

void DISPLAY_SetText(char *text, uint8_t line);
void DISPALY_Clear();
