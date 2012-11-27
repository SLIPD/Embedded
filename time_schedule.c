#include "time_schedule.h"

#include "efm32_timer.h"

#include <stdlib.h>

/* variables */

/* prototypes */

/* functions */
void TS_Init(schedule_t *schedule, TIMER_TypeDef *timer, uint8_t cc)
{
	
	schedule->start = NULL;
	schedule->current = NULL;
	schedule->last_action = NULL;
	schedule->timer = timer;
	schedule->timerCC = cc;
	
}

void TS_Insert(schedule_t *schedule, action_t *action)
{
	
	action_t *cur = schedule->start;
	if (action->time < cur->time)
	{
		action->next = cur;
		schedule->start = action;
		return;
	}
	
	while (cur->next != NULL && cur->next->time < action->time)
	{
		cur = cur->next;
	}
	
	if (cur->next == NULL)
	{
		action->next = NULL;
		cur->next = action;
	}
	else
	{
		action->next = cur->next;
		cur->next = action;
	}
	
}

void TS_Complete(schedule_t *schedule)
{
	
	schedule->current = schedule->start;
	
	TIMER_InitCC_TypeDef timerCCInit = 
	{
		.cufoa      = timerOutputActionNone,
		.cofoa      = timerOutputActionNone,
		.cmoa       = timerOutputActionNone,
		.mode       = timerCCModeCompare,
		.filter     = true,
		.prsInput   = false,
		.coist      = false,
		.outInvert  = false,
	};
	TIMER_InitCC(schedule->timer, schedule->timerCC, &timerCCInit);
	
	TIMER_CompareSet(schedule->timer, schedule->timerCC, schedule->current->time);
	
}

void TS_Update(schedule_t *schedule)
{
	
	uint32_t current_time = TIMER_CounterGet(schedule->timer);
	
	if (current_time < schedule->current->time)
	{
		current_time += TIMER_TopGet(schedule->timer);
	}
	
	do
	{
		
		schedule->current->action();
		schedule->current = schedule->current->next;
		if (schedule->current == NULL)
		{
			schedule->current = schedule->start;
			current_time -= TIMER_TopGet(schedule->timer);
		}
		
	}
	while (schedule->current->time <= current_time);
	
	TIMER_CompareSet(schedule->timer, schedule->timerCC, schedule->current->time);
	TRACE("#%i: NEXT SCHEDULED IRQ\n", TIMER_CounterGet(schedule->timer));
	
}
