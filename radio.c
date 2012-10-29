#include "radio.h"

#include "NRF24L01.h"
#include "efm32_usart.h"
#include "efm32_gpio.h"
#include "efm32_rtc.h"

#include "led.h"
#include "trace.h"
#include "node_config.h"
#include "flash.h"

/* variables */
uint8_t interrupts = 0;

/* prototypes */
uint8_t readRegister(uint8_t reg);
uint8_t readByte(uint8_t cmd);
void sendCommand(uint8_t cmd, uint8_t data);
void writeRegister(uint8_t reg, uint8_t data);
void sendPayload(uint8_t reg, uint8_t bytes, uint8_t *data);
void receivePayload(uint8_t cmd, uint8_t bytes, uint8_t *buf);
void writeRegisterMulti(uint8_t reg, uint8_t bytes, uint8_t *data);

void configSend();
void configRecv();
void configBoth();
void enableRX();
void enableTX();

/* functions */
void RADIO_Init()
{

	USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;

	/* Pin PC9 is configured to Push-pull */
	GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_PUSHPULL;
	/* Pin PC10 is configured to Input enabled */
	GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE4_MASK) | GPIO_P_MODEL_MODE4_INPUT;
	/* Pin PC11 is configured to Push-pull */
	GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_PUSHPULL;

	GPIO->P[1].DOUT &= ~(1 << 1);
	GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | GPIO_P_MODEL_MODE1_PUSHPULL;
	GPIO->P[1].DOUT |= (1 << 2);
	GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_PUSHPULL;
	GPIO->P[1].MODEL = (GPIO->P[1].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | GPIO_P_MODEL_MODE0_INPUT;
	
	// en rx pin
	GPIO->P[0].DOUT &= ~(1 << 2);
	GPIO->P[0].MODEL = (GPIO->P[0].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_PUSHPULL;
	
	USART2->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	
	usartInit.msbf = true;
	usartInit.clockMode = usartClockMode0;
	usartInit.baudrate = 7000000;
	USART_InitSync(RADIO_USART, &usartInit);
	RADIO_USART->ROUTE = (RADIO_USART->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | RADIO_USART_LOCATION;
	RADIO_USART->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;
	
	// configure gpio interrupt
	GPIO_IntClear(1 << 0);
	writeRegister(NRF_STATUS,0x70);
	GPIO_IntConfig(gpioPortB,0,false,true,true);
	
	// configure radio
	configBoth();
	
	// enable amp
	GPIO->P[0].DOUT |= (1 << 2);
	
	uint8_t status = readRegister(NRF_STATUS);
	
	if (status == 0x0E)
	{
		TRACE("GOOD response from RADIO\n");
	}
	else
	{
		TRACE("BAD response from RADIO\n");
	}
	
	#ifdef GPS
	enableTX();
	#elif defined(BS)
	enableRX();
	#endif
	
}

void configBoth()
{
	
	uint8_t addr_array[5];
	
	addr_array[0] = 0xE7;
	addr_array[1] = 0xE7;
	addr_array[2] = 0xE7;
	addr_array[3] = 0xE7;
	addr_array[4] = 0xE7;
	
	//writeRegister(NRF_CONFIG,0x0E);
	writeRegister(NRF_EN_AA,0x01);
	writeRegister(NRF_EN_RXADDR,0x3F);
	writeRegister(NRF_SETUP_AW,0x03);
	writeRegister(NRF_SETUP_RETR,0x70);
	writeRegister(NRF_RF_CH,NODE_CH);
	writeRegister(NRF_RF_SETUP,0x0F);
	
	writeRegisterMulti(NRF_TX_ADDR, 5, addr_array);
	writeRegisterMulti(NRF_RX_ADDR_P0, 5, addr_array);
	
	writeRegister(NRF_RX_PW_P0,0x20);
	writeRegister(NRF_DYNPD, 0x00);
	writeRegister(NRF_FEATURE, 0x00);
	
	// flush receive buffer, clear interupts
	sendCommand(NRF_FLUSH_TX, NRF_NOP);
	sendCommand(NRF_FLUSH_RX, NRF_NOP);
	writeRegister(NRF_STATUS,0x70);
	
}

void enableRX()
{
	writeRegister(NRF_CONFIG,0x3F);
	sendCommand(NRF_FLUSH_TX, NRF_NOP);
	sendCommand(NRF_FLUSH_RX, NRF_NOP);
	writeRegister(NRF_STATUS,0x70);
	RADIO_CE_hi;
}

void enableTX()
{
	RADIO_CE_lo;
	writeRegister(NRF_CONFIG,0x4E);
	sendCommand(NRF_FLUSH_TX, NRF_NOP);
	sendCommand(NRF_FLUSH_RX, NRF_NOP);
	writeRegister(NRF_STATUS,0x70);
}

void configSend()
{
	
	uint8_t addr_array[5];
	
	writeRegister(NRF_CONFIG,0x4E);
	writeRegister(NRF_EN_AA,0x00);
	writeRegister(NRF_EN_RXADDR,0x3F);
	writeRegister(NRF_SETUP_AW,0x03);
	writeRegister(NRF_SETUP_RETR,0x00);
	writeRegister(NRF_RF_CH,NODE_CH);
	writeRegister(NRF_RF_SETUP,0x0F);
	writeRegister(NRF_STATUS,0x70);
	
	addr_array[0] = 0xE7;
	addr_array[1] = 0xE7;
	addr_array[2] = 0xE7;
	addr_array[3] = 0xE7;
	addr_array[4] = 0xE7;
	
	writeRegisterMulti(NRF_TX_ADDR, 5, addr_array);
	writeRegister(NRF_DYNPD, 0x00);
	writeRegister(NRF_FEATURE, 0x00);
	
	// flush TX buffer
	sendCommand(NRF_FLUSH_TX, NRF_NOP);
	sendCommand(NRF_FLUSH_RX, NRF_NOP);
	writeRegister(NRF_STATUS,0x70);
	
}

void configRecv()
{

	uint8_t addr_array[5];
	
	writeRegister(NRF_CONFIG,0x3F);
	writeRegister(NRF_EN_AA,0x00);
	writeRegister(NRF_EN_RXADDR,0x3F);
	writeRegister(NRF_SETUP_AW,0x03);
	writeRegister(NRF_SETUP_RETR,0x00);
	writeRegister(NRF_RF_CH,NODE_CH);
	writeRegister(NRF_RF_SETUP,0x0F);
	
	addr_array[0] = 0xE7;
	addr_array[1] = 0xE7;
	addr_array[2] = 0xE7;
	addr_array[3] = 0xE7;
	addr_array[4] = 0xE7;
	
	writeRegisterMulti(NRF_RX_ADDR_P0, 5, addr_array);
	writeRegister(NRF_RX_PW_P0,0x20);
	writeRegister(NRF_DYNPD, 0x00);
	writeRegister(NRF_FEATURE, 0x00);
	
	// flush receive buffer, clear interupts
	sendCommand(NRF_FLUSH_TX, NRF_NOP);
	sendCommand(NRF_FLUSH_RX, NRF_NOP);
	writeRegister(NRF_STATUS,0x70);
	
	RADIO_CE_hi;
	
}

uint8_t readByte(uint8_t cmd)
{
  volatile uint8_t blah;
  RADIO_CSN_lo;
  while((RADIO_USART->STATUS & USART_STATUS_RXDATAV)) {
    blah = RADIO_USART->RXDATA;
  }
  while(!(RADIO_USART->STATUS & USART_STATUS_TXBL));
  RADIO_USART->TXDATA = cmd;
  while(!(RADIO_USART->STATUS & USART_STATUS_TXC));
  USART_Rx(RADIO_USART);
  RADIO_USART->TXDATA = 0x00;
  while (!(RADIO_USART->STATUS & USART_STATUS_TXC)) ;
  RADIO_CSN_hi;
  return USART_Rx(RADIO_USART);
}

uint8_t readRegister(uint8_t reg)
{
	return readByte(reg | NRF_R_REGISTER);
}

void writeRegister(uint8_t reg, uint8_t data)
{
  sendCommand((reg | NRF_W_REGISTER), data);
}

void writeRegisterMulti(uint8_t reg, uint8_t bytes, uint8_t *data)
{
  sendPayload((NRF_W_REGISTER | reg), bytes, data);
}

void sendPayload(uint8_t reg, uint8_t bytes, uint8_t *data)
{
  RADIO_CSN_lo;
  
  volatile uint8_t blah;
  while(!(RADIO_USART->STATUS & USART_STATUS_TXBL));
  RADIO_USART->TXDATA = reg;
  while(!(RADIO_USART->STATUS & USART_STATUS_TXC)) ;
  blah = RADIO_USART->RXDATA;
  int i;
  for (i = 0; i < bytes; i++) {
    while(!(RADIO_USART->STATUS & USART_STATUS_TXBL));
	  RADIO_USART->TXDATA = data[i];
    while(!(RADIO_USART->STATUS & USART_STATUS_TXC)) ;
    blah = RADIO_USART->RXDATA;
  }
  
  RADIO_CSN_hi;
}

void receivePayload(uint8_t cmd, uint8_t bytes, uint8_t *buf)
{
  RADIO_CSN_lo;
  RADIO_USART->CMD = USART_CMD_CLEARRX;
  USART_Tx(RADIO_USART, cmd);
  USART_Rx(RADIO_USART);
  int i;
  for (i = 0; i < bytes; i++) {
	  USART_Tx(RADIO_USART, 0xFF);
	  buf[i] = USART_Rx(RADIO_USART);
  }
  //while(!(RADIO_USART->STATUS & USART_STATUS_TXC)) ;
  RADIO_CSN_hi;
}


void sendCommand(uint8_t cmd, uint8_t data)
{
  RADIO_CSN_lo;
  while(!(RADIO_USART->STATUS & USART_STATUS_TXBL));
  RADIO_USART->TXDATA = cmd;
  while(!(RADIO_USART->STATUS & USART_STATUS_TXC));
  USART_Rx(RADIO_USART);
  RADIO_USART->TXDATA = data;
  while (!(RADIO_USART->STATUS & USART_STATUS_TXC)) ;
  RADIO_CSN_hi;
}  

void RADIO_Main()
{
	
	uint8_t packet[RADIO_PACKET_SIZE], fifo_status;
	
	if (interrupts > 0)
	{
		
		LED_Off(BLUE);
		
		fifo_status = readRegister(NRF_FIFO_STATUS);
		
		while (!(fifo_status & 0x01))
		{
			
			receivePayload(NRF_R_RX_PAYLOAD,RADIO_PACKET_SIZE,packet);
			
			TRACE(&packet[1]);
			
		}
		
	}
	
}

void RADIO_Interrupt()
{
	
	// if interrupt from radio
	if (GPIO->IF & (1 << RADIO_INT_PIN))
	{
		
		//TRACE("Radio: INTERRUPT\n");
		
		// read status
		uint8_t status = readRegister(NRF_STATUS);
		
		#ifdef BS
		// if packet received
		if (status & 0x40)
		{
			//TRACE("Radio: packet received\n");
			interrupts++;
			LED_On(BLUE);
		}
		#endif
		
		#ifdef GPS
		// if packet sent
		if (status & 0x20)
		{
			//TRACE("Radio: packet successfully transmitted\n");
			
			uint8_t fifo_status = readRegister(NRF_FIFO_STATUS);
			if (fifo_status & 0x10)
				RADIO_CE_lo;
			LED_Off(BLUE);
			LED_Off(RED);
		}
		
		// packet not sent
		if (status & 0x10)
		{
			//TRACE("Radio: could not send packet\n");
			RADIO_CE_lo;
			LED_Off(BLUE);
			LED_On(RED);
		}
		#endif
		
		// clear interrupts
		GPIO->IFC = (1 << RADIO_INT_PIN);
		writeRegister(NRF_STATUS,0x70);
		
	}
	
}

void RADIO_Transmit(uint8_t *packet)
{
	
	uint8_t flash_packet[RADIO_PACKET_SIZE];
	uint8_t fifo_status  = readRegister(NRF_FIFO_STATUS);
	
	// if data in flash, send that first
	while ((!(fifo_status & (1 << 5))) && FLASH_Pop(flash_packet))
	{
		
		sendPayload(NRF_W_TX_PAYLOAD,RADIO_PACKET_SIZE,flash_packet);
		TRACE("CATCH UP PACKET\n");
		fifo_status  = readRegister(NRF_FIFO_STATUS);
		
	}
	
	// block while FIFO full
	if (fifo_status & (1 << 5))
	{
		// write to flash
		FLASH_Push(packet);
		TRACE("PUSHING PACKET\n");
	}
	else
	{
		sendPayload(NRF_W_TX_PAYLOAD,RADIO_PACKET_SIZE,packet);
		TRACE("Radio: transmitting packet(s)\n");
	}
	
	// send payload to chip
	
	
	LED_On(BLUE);
	
	// enable the chip to send the packet
	RADIO_CE_hi;
	
}
