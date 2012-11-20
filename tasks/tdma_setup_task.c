#include "tdma_setup_task.h"

#include "system.h"
#include "tasks.h"

#include "scheduler.h"

#include "efm32_rtc.h"

#include "radio_init_task.h"

void tdma_setup_task_entrypoint()
{
	
	RADIO_Enable(OFF);
	RADIO_SetMode(RX);
	RADIO_Enable(RX);
	
	uint32_t time = 0;
	
	while(1)
	{
		
		if (RADIO_RxBufferSize())
		{
			time = RTC_CounterGet();
		}
		
		if (RADIO_TxBufferSize())
		{
			
			char tmsg[255];
			sprintf(tmsg,"size: %i\n", RADIO_TxBufferSize());
			TRACE(tmsg);
			
			if (RTC_CounterGet() - time > TX_WAIT)
			{
				
				RADIO_Enable(OFF);
				RADIO_SetMode(TX);
				RADIO_SetAutoRefil(true);
				RADIO_TxBufferFill();
				RADIO_Enable(TX);
				
				while (RADIO_Sending())
				{
					SCHEDULER_Yield();
				}
				
				RADIO_Enable(OFF);
				RADIO_SetMode(RX);
				RADIO_Enable(RX);
				
			}
			
		}
		
	}
	
}
