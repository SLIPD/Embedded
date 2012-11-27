#include "basestation.h"

#include <string.h>

#include "packets.h"
#include "radio.h"
#include "gps.h"

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
		pibound_position = 32,
		meshbound_position = 0;
	bool waiting = true,
		gps_enable = false,
		tx = false;
	
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
	
	while (!(UART1->STATUS & UART_STATUS_TXBL));
	UART1->TXDATA = '*';
	
	if (gps_enable)
	{
		Packet p;
	
		// wait for GPS fix
		GPS_GetFix();
		
		// wait for precise fix
		GPS_GetPrecision(30);
		
		GPS_Vector_Type gpsv;
		GPS_Read(&gpsv);
		
		// send GPS fix packet to pi
		
		p.originId = 0x00;
		p.destinationId = 0x00;
		p.ttl = 0x01;
		p.msgType = 0x01;
		
		p.payload.nodePosition.latitude = gpsv.lat;
		p.payload.nodePosition.longitude = gpsv.lon;
		p.payload.nodePosition.elevation = gpsv.alt/10;
		p.payload.nodePosition.hexaseconds = 0;
		
		p.payload.nodePosition.last_seq_num = 0;
		
		pibound_position = 0;
		memcpy(&pibound_packet,&p,32);
	}
	else
	{
		basestation_echo();
	}
	
	RADIO_Enable(OFF);
	RADIO_SetMode(RX);
	RADIO_Enable(RX);
	
	while (1)
	{
		
		RADIO_HandleMessages();
		
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
		
		if (UART1->STATUS & UART_STATUS_TXBL)
		{
			
			if (pibound_position == 32)
			{
				
				if (RADIO_Recv(pibound_packet))
				{
					pibound_position = 0;
				}
				else
				{
					continue;
				}
				
			}
			
			UART1->TXDATA = pibound_packet[pibound_position++];
			
		}
		
	}
	
}

void basestation_echo()
{
	
	uint8_t p[32],
		position = 0;
	
	while (position < 32)
	{
		if (UART1->STATUS & UART_STATUS_RXDATAV)
			p[position++] = UART1->RXDATA;
	}
	
	position = 0;
	
	while (position < 32)
	{
		if (UART1->STATUS & UART_STATUS_TXBL)
			UART1->TXDATA = p[position++];
	}
	
}

bool basestation_handlePacket(Packet *p)
{
	
	if (p->originId == 0x00 &&
		p->destinationId == 0x00)
	{
		
		switch (p->msgType)
		{
		case 0x00:
			RADIO_ConfigTDMA(*p);
			pre_tdma = false;
			RADIO_EnableTDMA();
			break;
		}
		
		return true;
	}
	else
	{
		return false;
	}
	
}
