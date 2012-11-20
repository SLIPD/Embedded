
#include "system.h"
#include "tasks.h"

#include "radio_init_task.h"
#include "led.h"

/* variables */
uint8_t partial_packet[32],
	position = 0;

/* prototypes */
void basestation_recv_uart(uint8_t byte);

/* functions */
void basestation_recv_uart(uint8_t byte)
{
	partial_packet[position] = byte;
	position++;
	if (position == 32)
	{
		position = 0;
		RADIO_Send(partial_packet);
	}
}

void basestation_radio_task_entrypoint()
{
	
	uint8_t packet[32];
	
	UART1_SetRecvHandler(basestation_recv_uart);
	
	while(1)
	{
		
		if (RADIO_Recv(packet))
		{
			
			UART1_Send(packet,32);
			
		}
		
	}
	
}
