#include "display.h"

#include "efm32.h"

#include "efm32_cmu.h"
#include "efm32_rtc.h"
#include "efm32_usart.h"

#include "led.h"

#include "efm32_i2c.h"
#include "i2cdrv.h"
#include "trace.h"

#include "string.h"

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

	return;
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
		wait_i2c(100);
		
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

void writeMessage(char* msg1, char* msg2)
{
	
	I2C_TransferSeq_TypeDef seq;

	uint8_t control = 0x40;

  seq.addr = 0x7C;      // Parameter Address
  seq.flags = I2C_FLAG_WRITE_WRITE;     // Indicate combined write / read seq 
  /* Select register to be read */
  // Format: Reg ID, Value
  // Length must be specified
  seq.buf[0].data = &control;
  seq.buf[0].len = 1;
  seq.buf[1].data = msg1;
  seq.buf[1].len = strlen(msg1);
  
  I2CDRV_Transfer(&seq);
  
  transfer(0x00,0xC0);
  
  seq.buf[1].data = msg2;
  seq.buf[1].len = strlen(msg2);
  
  I2CDRV_Transfer(&seq);
	
}

writeChar(uint8_t pos, uint8_t line, char msg)
{
	
	pos &= 0x80;
	if (line)
		pos &= 0x40;
	
	transfer(0x00,pos);
	
	I2C_TransferSeq_TypeDef seq;

	uint8_t control = 0x40;

  seq.addr = 0x7C;      // Parameter Address
  seq.flags = I2C_FLAG_WRITE_WRITE;     // Indicate combined write / read seq 
  /* Select register to be read */
  // Format: Reg ID, Value
  // Length must be specified
  seq.buf[0].data = &control;
  seq.buf[0].len = 1;
  seq.buf[1].data = msg;
  seq.buf[1].len = strlen(msg);
  
  I2CDRV_Transfer(&seq);
	
}

void position(uint8_t pos, uint8_t line)
{
	
	pos |= 0x80;
	if (line)
		pos |= 0x40;
	
	transfer(0x00,pos);
	
}

int showCursor()
{
	
	
	TRACE("INIT DISPLAY\n");
	//while (!(UART1->STATUS & UART_STATUS_RXDATAV));
	
	
	transfer(0x00,0x38);
	
	transfer(0x00,0x39);
	
	transfer(0x00,0x14);
	
	transfer(0x00,0x74);
	
	transfer(0x00,0x54);
	
	transfer(0x00,0x6F);
	
	transfer(0x00,0x0C);
	
	transfer(0x00,0x01);
  
  TRACE("WRITING MESSAGE\n");
	
	volatile char c;
	uint8_t pos = 0, line = 0;
	while (1)
	{
		
		while (!(UART1->STATUS & UART_STATUS_RXDATAV));
		c = UART1->RXDATA;
		
		position(pos,line);
		
		if (c == '\n')
		{
			line = (line + 1) % 2;
			pos = 0;
			continue;
		}
		
		if (c == 0x08)
		{
			transfer(0x40,' ');
			pos = (pos + 15) % 16;
			if (pos == 15)
				line = (line + 1) % 2;
			position(pos,line);
			transfer(0x40,'_');
			continue;
		}
		
		transfer(0x40,c);
		
		pos++;
		
		if (pos == 16)
		{
			pos = 0;
			line = (line + 1) % 2;
		}
		
		position(pos,line);
		transfer(0x40,'_');
		
	}
	
	//writeMessage("  >>> KITB <<<","  LOVES BOYS");
	
	while(1);
	
	//writeMessage("kit - do slip");
	transfer(0x00,0xC4);
	//writeMessage("TONIGHT");
	
	/*
	LED_On(BLUE);
	
	char* msg = "hello world";
	uint8_t len = strlen(msg);
	
	I2C_TransferSeq_TypeDef seq;
	
	uint8_t control = 0x40;
	
  seq.addr = 0x7C;      // Parameter Address
  seq.flags = I2C_FLAG_WRITE_WRITE;     // Indicate combined write / read seq 

  // Format: Reg ID, Value
  // Length must be specified
  seq.buf[0].data = &control;
  seq.buf[0].len = 1;
  seq.buf[1].data = &msg;
  seq.buf[1].len = len;
 
	I2CDRV_Transfer(&seq);
  */
  TRACE("DONE\n");
  
  LED_On(GREEN);
  
  while(1);

  return(0);
	
}

void write(uint8_t instruction, uint8_t *data, uint8_t len)
{
	
}

void DISPLAY_Init()
{
	
	showCursor();
	
}

void DISPLAY_SetText(char *text, uint8_t line);
void DISPALY_Clear();
