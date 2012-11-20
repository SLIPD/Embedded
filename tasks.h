#ifndef __TASKS_H__
#define __TASKS_H__

#include "scheduler.h"

/* tasks */
task_t radio_init_task,
	basestation_radio_task,
	tdma_task,
	tdma_setup_task,
	node_radio_task,
	display_init_task;

/* entry points */
void radio_init_task_entrypoint();
void basestation_radio_task_entrypoint();
void tdma_task_entrypoint();
void tdma_setup_task_entrypoint();
void node_radio_task_entrypoint();
void display_init_task_entrypoint();

#endif