#include "flash.h"

#include "stdbool.h"

#include "efm32_gpio.h"
#include "efm32_usart.h"

#include "trace.h"

/* variables */
uint32_t read_addr = 0, write_addr = 0;

/* prototypes */
void WREN(bool enable);
uint8_t RDSR();
void PP(uint8_t addr[3], uint8_t len, uint8_t payload[]);
void READ(uint8_t addr[3], uint8_t len, uint8_t payload[]);
void WRITE(uint8_t addr[3], uint8_t len, uint8_t payload[]);

/* functions */
// must be run after radio is init'd
void FLASH_Init()
{
	
	GPIO->P[1].DOUT |= (1 << 6);
	GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE6_MASK) | GPIO_P_MODEL_MODE6_PUSHPULL;
	
	// flash test
	uint8_t packet[32];
	int i;
	for (i = 1; i < 32; i++)
	{
		packet[i-1] = i;
	}
	packet[31] = 0;
	
	TRACE(packet);
	
	FLASH_Push(packet);
	
	TRACE("\n");
	
	for (i = 0; i < 32; i++)
	{
		packet[i] = 0xFF;
	}
	
	FLASH_Pop(packet);
	for (i = 0; i < 32; i++)
	{
		if(packet[i] == 0x00)
			packet[i] = 0xFF;
	}
	TRACE(packet);
	
	while (1);
	
}

void WRITE(uint8_t addr[3], uint8_t len, uint8_t payload[])
{
	
	do
	{
		WREN(true);
	}
	while (!(RDSR() & 0x02));
	
	PP(addr,len,payload);
	
	WREN(false);
	
}

void WREN(bool enable)
{
	
	GPIO->P[1].DOUT &= ~(1 << 6);
	
	USART2->CMD = USART_CMD_CLEARRX;
	
  while(!(USART2->STATUS & USART_STATUS_TXBL));
  if (enable)
		USART2->TXDATA = 0x06;
	else
		USART2->TXDATA = 0x04;
  while(!(USART2->STATUS & USART_STATUS_TXC));
  
  GPIO->P[1].DOUTSET = (1 << 6);
	
}

uint8_t RDSR()
{
	
	GPIO->P[1].DOUT &= ~(1 << 6);
	
	USART2->CMD = USART_CMD_CLEARRX;
	
  while(!(USART2->STATUS & USART_STATUS_TXBL));
  USART2->TXDATA = 0x05;
  while(!(USART2->STATUS & USART_STATUS_TXC));
  USART_Rx(USART2);
  USART2->TXDATA = 0x00;
  while (!(USART2->STATUS & USART_STATUS_TXC)) ;
  
  GPIO->P[1].DOUTSET = (1 << 6);
  
  return USART_Rx(USART2);
	
}

void PP(uint8_t addr[3], uint8_t len, uint8_t payload[])
{
	
	GPIO->P[1].DOUT &= ~(1 << 6);
	
	USART2->CMD = USART_CMD_CLEARRX;
	
  while(!(USART2->STATUS & USART_STATUS_TXBL));
  USART2->TXDATA = 0x02;
  while(!(USART2->STATUS & USART_STATUS_TXC));
  
  int i;
  for (i = 0; i < 3; i++)
  {
		USART_Rx(USART2);
		USART2->TXDATA = addr[i];
		while (!(USART2->STATUS & USART_STATUS_TXC)) ;
  }
  
  
  for (i = 0; i < len; i++)
  {
		USART_Rx(USART2);
		USART2->TXDATA = payload[i];
		while (!(USART2->STATUS & USART_STATUS_TXC)) ;
  }
  
  USART2->CMD = USART_CMD_CLEARRX;
  
  GPIO->P[1].DOUTSET = (1 << 6);
	
}

void READ(uint8_t addr[3], uint8_t len, uint8_t payload[])
{

	GPIO->P[1].DOUT &= ~(1 << 6);
	
	USART2->CMD = USART_CMD_CLEARRX;
	
  while(!(USART2->STATUS & USART_STATUS_TXBL));
  USART2->TXDATA = 0x03;
  while(!(USART2->STATUS & USART_STATUS_TXC));
  USART_Rx(USART2);
  
  int i;
  for (i = 0; i < 3; i++)
  {
		USART2->TXDATA = addr[i];
		while (!(USART2->STATUS & USART_STATUS_TXC));
		USART_Rx(USART2);
  }
  
  for (i = 0; i < len; i++)
  {
		USART2->TXDATA = 0x00;
		while (!(USART2->STATUS & USART_STATUS_TXC));
		payload[i] = USART_Rx(USART2);
  }
  
  GPIO->P[1].DOUTSET = (1 << 6);
  
  USART2->CMD = USART_CMD_CLEARRX;
	
}

void FLASH_Push(uint8_t *payload)
{
	
	uint8_t addr[3];
	addr[0] = 0xFF & (write_addr);
	addr[1] = 0xFF & (write_addr >> 2);
	addr[2] = 0XFF & (write_addr >> 4);
	
	WRITE(addr,FLASH_PACKET_SIZE,payload);
	write_addr = ((write_addr + FLASH_PACKET_SIZE) % FLASH_LENGTH);
	
}

uint8_t FLASH_Pop(uint8_t *payload)
{
	
	if (read_addr == write_addr)
	{
		return 0;
	}
	
	uint8_t addr[3];
	addr[0] = 0xFF & (read_addr);
	addr[1] = 0xFF & (read_addr >> 2);
	addr[2] = 0XFF & (read_addr >> 4);
	
	READ(addr,FLASH_PACKET_SIZE,payload);
	read_addr = ((read_addr + FLASH_PACKET_SIZE) % FLASH_LENGTH);
	
	return 1;
	
}