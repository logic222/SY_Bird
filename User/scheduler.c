#include "scheduler.h"
#include "stm32f10x.h"
#include "led.h"
#include "usart.h"
#include "delay.h"
#include "ppm.h"
#include "servo_motor.h"
#include "mpu6050.h"
#include "flightcontrol_b.h"
uint8_t task_num;
uint8_t led_state;
volatile uint32_t sysTickUptime = 0;

uint32_t GetTick(void) {
    return sysTickUptime;
}



// 任务列表
static scheduler_task_t scheduler_task[] = {
	{mpu6050_update,1,0},
	//{control_loop,5,0},
	{servo_motor_speed_update,5,0},
	//{ppm_update,1,0},
	//{ppm_send,1000,0},
    {led_blink, 500, 0},
    //{Serial_SendTest, 200, 0},
	
};

void Scheduler_Init(void) {
    task_num = sizeof(scheduler_task) / sizeof(scheduler_task_t);
}

void Scheduler_run(void) {
    
	for(int i = 0; i < task_num; i++)
	{
		uint32_t now_time = GetTick();
		if(now_time-scheduler_task[i].last_run >= scheduler_task[i].rate_ms)
		{
			scheduler_task[i].last_run = now_time;
			scheduler_task[i].task_func();
		}
	}
}

