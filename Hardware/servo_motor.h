#ifndef __SERVO_MOTOR_H
#define __SERVO_MOTOR_H

#include "stm32f10x.h"                  // Device header
#include "delay.h"
#define DELAY_MS 50      // 步进间隔时间(ms)
void TIM2_PWM_init(void);
void servo_motor_init(void);
void servo_motor_speed_update(void);
void Servo_Sweep(void);
int map(int x, int in_min, int in_max, int out_min, int out_max);
int constrain(int value, int min_val, int max_val);
void Print_PWM_Values(void);
#endif
