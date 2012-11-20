
#include "system.h"
#include "tasks.h"

/* variables */


/* prototypes */


/* functions */
void node_radio_task_entrypoint()
{
	
	uint8_t packet[32];
	
	// set up timer
	
	
	while(1)
	{
		
		if (RADIO_Recv(packet))
		{
			
			
			
		}
		
	}
	
}
