
#include "system.h"
#include "tasks.h"

#include "packets.h"
#include "radio_init_task.h"

/* variables */


/* prototypes */


/* functions */
void node_radio_task_entrypoint()
{
	
	Packet packet;
	
	// set up timer
	
	
	while(1)
	{
		
		if (RADIO_Recv((uint8_t*)&packet))
		{
			
			if (packet.payload.identification.id0 == *((uint32_t*)(0xFE081F0)) &&
				packet.payload.identification.id1 == *((uint32_t*)(0xFE081F4)))
			{
				
				// disable timer
				
				
				// move to TDMA
				
				
			}
			
		}
		
	}
	
}

/*
ident.payload.identification.id0 = *((uint32_t*)(0xFE081F0));
ident.payload.identification.id1 = *((uint32_t*)(0xFE081F4));
*/
