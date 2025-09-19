#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"                  // Device header

typedef struct {
    // 外环PID参数
    float outer_kp;         // 外环比例系数
    float outer_ki;         // 外环积分系数
    float outer_kd;         // 外环微分系数
    
    // 内环PID参数
    float inner_kp;         // 内环比例系数
    float inner_ki;         // 内环积分系数
    float inner_kd;         // 内环微分系数
    
    // 外环状态变量
    float outer_target;     // 外环目标值
    float outer_feedback;   // 外环反馈值
    float outer_error;      // 外环当前误差
    float outer_error_last; // 外环上一次误差
    float outer_error_sum;  // 外环误差积分
    float outer_output;     // 外环输出(作为内环的目标值)
    
    // 内环状态变量
    float inner_feedback;   // 内环反馈值
    float inner_error;      // 内环当前误差
    float inner_error_last; // 内环上一次误差
    float inner_error_sum;  // 内环误差积分
    float inner_output;     // 内环输出(最终控制量)
    
    // 输出限幅
    float outer_output_max; // 外环输出最大值
    float outer_output_min; // 外环输出最小值
    float inner_output_max; // 内环输出最大值
    float inner_output_min; // 内环输出最小值
    
    // 积分限幅
    float outer_integral_max; // 外环积分限幅
    float inner_integral_max; // 内环积分限幅
    
    // 采样时间(单位:秒)
    float sample_time;
	
	float outer_integral_deadband; // 外环积分死区阈值
    float inner_integral_deadband; // 内环积分死区阈值
} CascadePID;
void CascadePID_Init(CascadePID *pid, 
                    float outer_kp, float outer_ki, float outer_kd,
                    float inner_kp, float inner_ki, float inner_kd,
                    float sample_time);

float CascadePID_Calculate(CascadePID *pid, float target, float outer_feedback, float inner_feedback);
void SetOuterPIDParams(CascadePID *pid, float kp, float ki, float kd);
void SetInnerPIDParams(CascadePID *pid, float kp, float ki, float kd);
void SetOutputLimits(CascadePID *pid, float outer_max, float outer_min, float inner_max, float inner_min);
void SetIntegralLimits(CascadePID *pid, float outer_max, float inner_max);

#endif
