#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "stm32f10x.h"                  // Device header

typedef struct
{
	void(*task_func)(void);
	uint16_t rate_ms;
	uint32_t last_run;
}scheduler_task_t;

uint32_t GetTick(void); 
void Scheduler_Init(void);
void Scheduler_run(void);

#endif
