#ifndef __TIME_SCHEDULE_H__
#define __TIME_SCEDULER_H__

#include <stdint.h>

#include "efm32_timer.h"

typedef struct action_t
{
	
	uint32_t time;
	void (*action)(void);
	struct action_t *next;
	
} action_t;

typedef struct
{
	
	TIMER_TypeDef *timer;
	uint32_t timerCC;
	action_t *start,
		*current;
	
} schedule_t;

void TS_Init(schedule_t *schedule, TIMER_TypeDef *timer, uint8_t cc);
void TS_Insert(schedule_t *schedule, action_t *action);
void TS_Complete(schedule_t *schedule);
void TS_Update(schedule_t *schedule);

#endif