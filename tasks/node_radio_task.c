
#include "system.h"
#include "tasks.h"

#include "packets.h"
#include "radio_init_task.h"
#include "led.h"

/* variables */


/* prototypes */
void node_send_ident_rt();

/* functions */
void node_send_ident_rt()
{
	
	Packet ident;
	
	ident.originId = 0xFF;
	ident.destinationId = 0x00;
	ident.ttl = 1;
	ident.msgType = 0x00;
	
	ident.payload.identification.id0 = *((uint32_t*)(0xFE081F0));
	ident.payload.identification.id1 = *((uint32_t*)(0xFE081F4));
	
	RADIO_Send((uint8_t*)&ident);
	
}

void node_radio_task_entrypoint()
{
	
	Packet packet;
	
	// set up timer
	TIMER_Reset(TIMER0);

	TIMER_TopSet(TIMER0, 48000000 / 1024);
	
	// enable timers 0
	TIMER_Init_TypeDef timerInit =
	{
		.enable     = true, 
		.debugRun   = true, 
		.prescale   = timerPrescale1024, 
		.clkSel     = timerClkSelHFPerClk, 
		.fallAction = timerInputActionNone, 
		.riseAction = timerInputActionNone, 
		.mode       = timerModeUp, 
		.dmaClrAct  = false,
		.quadModeX4 = false, 
		.oneShot    = false, 
		.sync       = false, 
	};
	
	timer_cb_table_t callback;
	
	callback.timer = TIMER0;
	callback.flags = TIMER_IF_OF;
	callback.cb = node_send_ident_rt;
	
	TIMER_RegisterCallback(&callback);

	TIMER_Init(TIMER0, &timerInit);
	
	//display_write("Waiting...\n");
	
	while(1);
	{
		
		if (RADIO_Recv((uint8_t*)&packet))
		{
			
			TRACE("PACKET RECV'd\n");
			
			switch (packet.msgType)
			{
				case 0x00:
			
					if (packet.payload.identification.id0 == *((uint32_t*)(0xFE081F0)) &&
						packet.payload.identification.id1 == *((uint32_t*)(0xFE081F4)))
					{
						
						// disable timer
						timerInit.enable = false;
						TIMER_Init(TIMER0, &timerInit);
						
						TRACE("IDENT RECEIVED\n");
						
						RADIO_SetNodeId(packet.payload.identification.nodeInfo);
						
						display_write("ident received\n");
						
						while(1);
						
					}
					
					break;
				
				case 0x03:
					
					// show message
					
					
					break;
			}
			
		}
		
	}
	
}
