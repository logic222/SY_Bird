#include "PID.h"
#include <math.h>


/**
  * @brief  初始化串级PID控制器
  * @param  pid: PID结构体指针
  * @param  outer_kp: 外环比例系数
  * @param  outer_ki: 外环积分系数
  * @param  outer_kd: 外环微分系数
  * @param  inner_kp: 内环比例系数
  * @param  inner_ki: 内环积分系数
  * @param  inner_kd: 内环微分系数
  * @param  sample_time: 采样时间(秒)
  * @retval 无
  */
void CascadePID_Init(CascadePID *pid, 
                    float outer_kp, float outer_ki, float outer_kd,
                    float inner_kp, float inner_ki, float inner_kd,
                    float sample_time)
{
    // 设置PID参数
    pid->outer_kp = outer_kp;
    pid->outer_ki = outer_ki;
    pid->outer_kd = outer_kd;
    
    pid->inner_kp = inner_kp;
    pid->inner_ki = inner_ki;
    pid->inner_kd = inner_kd;
    
    // 初始化状态变量
    pid->outer_target = 0.0f;
    pid->outer_feedback = 0.0f;
    pid->outer_error = 0.0f;
    pid->outer_error_last = 0.0f;
    pid->outer_error_sum = 0.0f;
    pid->outer_output = 0.0f;

    pid->inner_feedback = 0.0f;
    pid->inner_error = 0.0f;
    pid->inner_error_last = 0.0f;
    pid->inner_error_sum = 0.0f;
    pid->inner_output = 0.0f;

    // 设置默认输出限幅
    pid->outer_output_max = 500.0f;
    pid->outer_output_min = -500.0f;
    pid->inner_output_max = 500.0f;
    pid->inner_output_min = -500.0f;

    // 设置默认积分限幅
    pid->outer_integral_max = 30.0f;
    pid->inner_integral_max = 30.0f;

    // 设置采样时间
    pid->sample_time = sample_time;

    // 死区设置
    pid->outer_integral_deadband = 0.5f;
    pid->inner_integral_deadband = 0.2f;
}


/**
  * @brief  串级PID计算
  * @param  pid: PID结构体指针
  * @param  target: 目标值(外环输入)
  * @param  outer_feedback: 外环反馈值
  * @param  inner_feedback: 内环反馈值
  * @retval PID输出值
  */
float CascadePID_Calculate(CascadePID *pid, float target, float outer_feedback, float inner_feedback)
{
    float outer_p = 0, outer_i = 0, outer_d = 0;
    float inner_p = 0, inner_i = 0, inner_d = 0;

    // 更新反馈值和目标值
    pid->outer_target = target;
    pid->outer_feedback = outer_feedback;
    pid->inner_feedback = inner_feedback;

    // ========== 外环 PID ==========
    pid->outer_error = pid->outer_target - pid->outer_feedback;

    // 当误差非常小时，清除积分（避免“微小误差积分饱和”）
	if (fabsf(pid->outer_error) < pid->outer_integral_deadband) {
		pid->outer_error_sum = 0;
}


    // 比例项
    outer_p = pid->outer_kp * pid->outer_error;

    // 条件积分（防止积分饱和导致系统卡住）
    float outer_output_temp = outer_p + pid->outer_ki * pid->outer_error_sum;
    if (outer_output_temp < pid->outer_output_max && outer_output_temp > pid->outer_output_min) {
        pid->outer_error_sum += pid->outer_error * pid->sample_time;
    }

    // 积分限幅
    if (pid->outer_error_sum > pid->outer_integral_max) pid->outer_error_sum = pid->outer_integral_max;
    if (pid->outer_error_sum < -pid->outer_integral_max) pid->outer_error_sum = -pid->outer_integral_max;

    outer_i = pid->outer_ki * pid->outer_error_sum;

    // 微分项
    outer_d = pid->outer_kd * (pid->outer_error - pid->outer_error_last) / pid->sample_time;
    pid->outer_error_last = pid->outer_error;

    // 输出
    pid->outer_output = outer_p + outer_i + outer_d;

    // 限幅
    if (pid->outer_output > pid->outer_output_max) pid->outer_output = pid->outer_output_max;
    if (pid->outer_output < pid->outer_output_min) pid->outer_output = pid->outer_output_min;

    // ========== 内环 PID ==========
    pid->inner_error = pid->outer_output - pid->inner_feedback;

    if (fabsf(pid->inner_error) < pid->inner_integral_deadband) {
		pid->inner_error_sum = 0;
	}


    // 比例项
    inner_p = pid->inner_kp * pid->inner_error;

    // 条件积分
    float inner_output_temp = inner_p + pid->inner_ki * pid->inner_error_sum;
    if (inner_output_temp < pid->inner_output_max && inner_output_temp > pid->inner_output_min) {
        pid->inner_error_sum += pid->inner_error * pid->sample_time;
    }

    // 积分限幅
    if (pid->inner_error_sum > pid->inner_integral_max) pid->inner_error_sum = pid->inner_integral_max;
    if (pid->inner_error_sum < -pid->inner_integral_max) pid->inner_error_sum = -pid->inner_integral_max;

    inner_i = pid->inner_ki * pid->inner_error_sum;

    // 微分项
    inner_d = pid->inner_kd * (pid->inner_error - pid->inner_error_last) / pid->sample_time;
    pid->inner_error_last = pid->inner_error;

    // 输出
    pid->inner_output = inner_p + inner_i + inner_d;

    // 限幅
    if (pid->inner_output > pid->inner_output_max) pid->inner_output = pid->inner_output_max;
    if (pid->inner_output < pid->inner_output_min) pid->inner_output = pid->inner_output_min;

    return pid->inner_output;
}


/**
  * @brief  设置外环PID参数
  * @param  pid: PID结构体指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  * @retval 无
  */
void SetOuterPIDParams(CascadePID *pid, float kp, float ki, float kd)
{
    pid->outer_kp = kp;
    pid->outer_ki = ki;
    pid->outer_kd = kd;
}

/**
  * @brief  设置内环PID参数
  * @param  pid: PID结构体指针
  * @param  kp: 比例系数
  * @param  ki: 积分系数
  * @param  kd: 微分系数
  * @retval 无
  */
void SetInnerPIDParams(CascadePID *pid, float kp, float ki, float kd)
{
    pid->inner_kp = kp;
    pid->inner_ki = ki;
    pid->inner_kd = kd;
}

/**
  * @brief  设置输出限幅
  * @param  pid: PID结构体指针
  * @param  outer_max: 外环最大输出
  * @param  outer_min: 外环最小输出
  * @param  inner_max: 内环最大输出
  * @param  inner_min: 内环最小输出
  * @retval 无
  */
void SetOutputLimits(CascadePID *pid, float outer_max, float outer_min, float inner_max, float inner_min)
{
    pid->outer_output_max = outer_max;
    pid->outer_output_min = outer_min;
    pid->inner_output_max = inner_max;
    pid->inner_output_min = inner_min;
}

/**
  * @brief  设置积分限幅
  * @param  pid: PID结构体指针
  * @param  outer_max: 外环最大积分
  * @param  inner_max: 内环最大积分
  * @retval 无
  */
void SetIntegralLimits(CascadePID *pid, float outer_max, float inner_max)
{
    pid->outer_integral_max = outer_max;
    pid->inner_integral_max = inner_max;
}
