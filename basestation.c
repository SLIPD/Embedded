#include "basestation.h"

#include <string.h>

#include "packets.h"
#include "radio.h"
#include "gps.h"
#include "led.h"
#include "trace.h"

/* variables */

/* prototypes */
bool basestation_handlePacket(Packet *p);

/* functions */
void basestation_main()
{
	
	#ifdef TEST
		
		LED_On(GREEN);
		LED_On(RED);
		LED_On(BLUE);
		
		uint8_t recv;
		do
		{
			while (!(UART1->STATUS & UART_STATUS_RXDATAV));
			recv = UART1->RXDATA;
		}
		while (recv != '#');
		
		LED_Off(RED);
		for (int i = 0; i < 100000; i++);
		
		while (!(UART1->STATUS & UART_STATUS_TXBL));
		UART1->TXDATA = '*';
		while (!(UART1->STATUS & UART_STATUS_TXC));
		
		uint8_t tmp_packet[32];
		
		for (int i = 0; i < 32; i++)
		{
			while (!(UART1->STATUS & UART_STATUS_RXDATAV));
			tmp_packet[i] = UART1->RXDATA;
		}
		
		for (int i = 0; i < 100000; i++);
		LED_Off(BLUE);
		
		TRACE_SendPayload(tmp_packet,32);
		
	#else
	
		LED_On(GREEN);
		LED_On(RED);
		LED_On(BLUE);
		
		uint8_t recv;
		do
		{
			while (!(UART1->STATUS & UART_STATUS_RXDATAV));
			recv = UART1->RXDATA;
		}
		while (recv != '*');
		
		LED_Off(RED);
		for (int i = 0; i < 100000; i++);
		
		while (!(UART1->STATUS & UART_STATUS_TXBL));
		UART1->TXDATA = '*';
		while (!(UART1->STATUS & UART_STATUS_TXC));
		
		Packet position;
		
		while (!GPS_GetFix());
		
		GPS_Vector_Type gps_position;
		
		while (!GPS_Read(&gps_position));
		
		position.originId = 0x00;
		position.destinationId = 0x00;
		position.ttl = 1;
		position.msgType = 0x01;
		position.payload.nodePosition.latitude = gps_position.lat;
		position.payload.nodePosition.longitude = gps_position.lon;
		position.payload.nodePosition.elevation = gps_position.alt;
		
		TRACE_SendPayload((uint8_t*)&position,32);
	
	#endif
	
	RADIO_Init();
	RADIO_Enable(OFF);
	RADIO_SetMode(RX);
	RADIO_Enable(RX);
	
	TRACE_Enable();
	
	LED_Off(GREEN);
	
	uint8_t meshbound_packet[32],
		meshbound_position = 0,
		pibound_packet[32];
	while(1)
	{
		
		RADIO_HandleMessages();

		if (UART1->STATUS & UART_STATUS_RXDATAV)
		{

			meshbound_packet[meshbound_position++] = UART1->RXDATA;

			if (meshbound_position == 32)
			{
				// check for packets for basestation

				if (!basestation_handlePacket((Packet*)&meshbound_packet))
					RADIO_Send(meshbound_packet);

				meshbound_position = 0;
			}

		}

		if (RADIO_Recv(pibound_packet))
		{
			TRACE_SendPayload(pibound_packet,32);
		}

	}
	
}

bool basestation_handlePacket(Packet *p)
{
	return false;
}
