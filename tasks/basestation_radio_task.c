
#include "system.h"
#include "tasks.h"

#include "radio_init_task.h"
#include "led.h"

/* variables */
uint8_t partial_packet[32],
	position = 0;
bool started = false;

/* prototypes */
void basestation_recv_uart(uint8_t byte);
void basestation_send_packet_rt();

/* functions */
void basestation_recv_uart(uint8_t byte)
{
	partial_packet[position] = byte;
	position++;
	if (position == 32)
	{
		position = 0;
		SCHEDULER_RunRTTask(basestation_send_packet_rt);
	}
}

void basestation_get_starter(uint8_t byte)
{
	if (byte == '*')
	{
		started = true;
	}
}

void basestation_send_packet_rt()
{
	RADIO_Send(partial_packet);
}

void basestation_radio_task_entrypoint()
{
	
	uint8_t packet[32];
	
	UART1_SetRecvHandler(basestation_get_starter);
	
	while(!started)
		SCHEDULER_Yield();
	
	char star = '*';
	UART1_Send((uint8_t*)&star,1);
	
	UART1_SetRecvHandler(basestation_recv_uart);
	
	while(1)
	{
		
		if (RADIO_Recv(packet))
		{
			
			UART1_Send(packet,32);
			
		}
		
	}
	
}
