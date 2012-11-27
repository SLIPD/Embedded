#include "basestation.h"

#include <string.h>

#include "packets.h"
#include "radio.h"
#include "gps.h"
#include "led.h"

/* variables */
bool pre_tdma = true;

/* prototypes */
bool basestation_handlePacket(Packet *p);
void basestation_echo();

void basestation_main()
{
	
	RADIO_Init();
	
	uint8_t pibound_packet[32],
		meshbound_packet[32],
		meshbound_position = 0;
	bool waiting = true,
		gps_enable = false,
		tx = false;
	
	LED_On(RED);
	LED_On(GREEN);
	LED_On(BLUE);
	
	while (waiting)
	{
		if (UART1->STATUS & UART_STATUS_RXDATAV)
		{
			uint8_t byte = UART1->RXDATA;
			if (byte == '*')
			{
				waiting = false;
				gps_enable = true;
			}
			if (byte == '#')
			{
				waiting = false;
				gps_enable = false;
			}
		}
	}
	
	LED_Off(BLUE);
	for (int i = 0; i < 100000; i++);
	
	while (!(UART1->STATUS & UART_STATUS_TXBL));
	UART1->TXDATA = '*';
	
	LED_Off(RED);
	for (int i = 0; i < 100000; i++);
	
	if (gps_enable)
	{
		Packet p;
	
		// wait for GPS fix
		GPS_GetFix();
		
		// wait for precise fix
		GPS_GetPrecision(100);
		
		GPS_Vector_Type gpsv;
		GPS_Read(&gpsv);
		
		// send GPS fix packet to pi
		
		p.originId = 0x00;
		p.destinationId = 0x00;
		p.ttl = 0x01;
		p.msgType = 0x01;
		
		memset(p.payload.nodePosition.positions,0,12*2);
		p.payload.nodePosition.positions[0].latitude = gpsv.lat;
		p.payload.nodePosition.positions[0].longitude = gpsv.lon;
		p.payload.nodePosition.positions[0].elevation = gpsv.alt/10;
		p.payload.nodePosition.positions[0].hexaseconds = 0;
		
		p.payload.nodePosition.last_seq_num = 0;
		
		memcpy(&pibound_packet,&p,32);
		TRACE_SendPayload(pibound_packet,32);
	}
	else
	{
		basestation_echo();
	}
	
	LED_Off(GREEN);
	TRACE_Enable();
	
	RADIO_Enable(OFF);
	RADIO_SetMode(RX);
	RADIO_Enable(RX);
	
	while (1)
	{
		
		RADIO_HandleMessages();
		/*
		if (pre_tdma)
		{
			if (RADIO_TxBufferSize())
			{
				
				if (!tx)
				{
				
					RADIO_Enable(OFF);
					RADIO_SetMode(TX);
					RADIO_TxBufferFill();
					RADIO_Enable(TX);
					tx = true;
					
				}
				
			}
			else
			{
				
				if (tx && !RADIO_Sending())
				{
					RADIO_Enable(OFF);
					RADIO_SetMode(RX);
					RADIO_Enable(RX);
					tx = false;
				}
				
			}
		}
		*/
		
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
		
		RADIO_HandleMessages();
		
		if (RADIO_Recv(pibound_packet))
			TRACE_SendPayload(pibound_packet,32);
		
	}
	
}

void basestation_echo()
{
	
	uint8_t p[32],
		position = 0;
	
	for (position = 0; position < 32; position++)
	{
		while (!(UART1->STATUS & UART_STATUS_RXDATAV));
		p[position] = UART1->RXDATA;
	}
	
	for (int i = 0; i < 100000; i++);
	
	TRACE_SendPayload(p,32);
	
}

bool basestation_handlePacket(Packet *p)
{
	
	if (p->originId == 0x00 &&
		p->destinationId == 0x00)
	{
		
		switch (p->msgType)
		{
		case 0x00:
			if (pre_tdma)
			{
				RADIO_ConfigTDMA(*p);
				//pre_tdma = false;
				RADIO_EnableTDMA();
			}
			break;
		}
		
		return true;
	}
	else
	{
		return false;
	}
	
}
