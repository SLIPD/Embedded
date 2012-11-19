#ifndef __TASKS_H__
#define __TASKS_H__

#include "scheduler.h"

/* tasks */
task_t radio_init_task,
	basestation_radio_task,
	tdma_task,
	noisy_channel_task;

/* entry points */
void radio_init_task_entrypoint();
void basestation_radio_task_entrypoint();
void tdma_task_entrypoint();
void noisy_channel_task_entrypoint();

#endif