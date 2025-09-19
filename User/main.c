#include "stm32f10x.h"
#include "scheduler.h"
#include "led.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "ppm.h"
#include "servo_motor.h"
#include "mpu6050.h"
#include "flightcontrol_b.h"
uint8_t ID;
int main(void) {
    // 初始化系统时钟和SysTick
    SystemInit();
	NVIC_SetPriority(SysTick_IRQn, 0);
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1);
    }
	MPU6050_Init();
    Scheduler_Init();
	TIM2_PWM_init();//好的
	servo_motor_init();
	PPM_Input_Init();
	LED_Init();
	MyUsart_Init();
	initialize_pid();
	GyroOffset_Init();
	//MPU6050_CalibrateGyro(1000);
	mpu6050_param_t mpu6050_data;  
	//ID = MPU6050_GetID();	
    while (1) {
        Scheduler_run();
		//updateAndCalculate();
		//get_eulerAngle();
		//mpu6050_update();
		//control_loop();
		//Mpu6050_AccData();
		//Mpu6050_GyroData();
		//Mpu6050_GetValues();
		//Serial_SendHex(ID);
		//Serial_SendString("\r\n");
		//MahonyAHRSupdate(&mpu6050_data);
		
		
		
		
    }
}







