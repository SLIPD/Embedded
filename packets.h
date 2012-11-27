#ifndef __PACKETS_H__
#define __PACKETS_H__

#include <stdint.h>

typedef struct 
{
	
	uint32_t id0;
	uint32_t id1;
	uint32_t tdma_gp;
	uint32_t tdma_txp;
	uint32_t tdma_txp_p;
	uint8_t nodeId;
	uint8_t nc;
	uint8_t c;
	
} MessageType_Identification;

typedef struct
{
	
	int32_t latitude;
	int32_t longitude;
	int16_t elevation;
        uint16_t hexaseconds;
	
	
	uint16_t last_seq_num;
	
} MessageType_NodePosition;

typedef struct
{
	
	uint16_t seq_num;
	struct
	{
		uint32_t latitude;
		uint32_t longitude;
	} waypoints[3];
	
} MessageType_Waypoint;

typedef struct
{
	
	uint8_t message[26];
	
} MessageType_Message;

typedef struct
{
	
	uint8_t originId;
	uint8_t destinationId;
	uint8_t ttl;
	uint8_t msgType;
	uint16_t timestamp __attribute__ ((packed));
	union
	{
		
		MessageType_Identification identification __attribute__ ((packed));
		MessageType_NodePosition nodePosition __attribute__ ((packed));
		MessageType_Waypoint waypoint __attribute__ ((packed));
		MessageType_Message message;
		
	} payload;
	
} Packet;

#endif