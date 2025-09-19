#include "ppm.h"
#include "flightcontrol_b.h"
#include "servo_motor.h"
#include "mpu6050.h"
#include "PID.h"
#include "usart.h"
#include "math.h"
#include <stdio.h>
extern uint16_t ppmValues[12];
extern int16_t rollint16,pitchint16,yawint16;//mpu6050实际测量角度(外环反馈值)
extern float roll_rate,yaw_rate,pitch_rate;//mpu6050得到的角速度(内环反馈值)
float target_pitch, target_roll, target_yaw;//ppm转换得到的目标角
float servoMiddlebias=9*1000/180;
float ppm_target_roll=0.0f,ppm_target_pitch=0.0f,ppm_target_yaw=0.0f;
uint16_t pwm_roll=1500;
uint16_t pwm_pitch=1500;
// 定义一个全局的 CascadePID 控制器变量
CascadePID pid_roll;
CascadePID pid_pitch;
CascadePID pid_yaw;

// 初始化 PID 参数
void initialize_pid(void)
{
    // 初始化 pid_roll
    CascadePID_Init(&pid_roll,
                    0.5f, 0.0f, 0.0f,   // outer_kp, ki, kd
                    1.0f, 0.0f, 0.0f,   // inner_kp, ki, kd
                    0.01f);            // sample_time
    pid_roll.outer_output_max = 1000.0f;
    pid_roll.outer_output_min = -1000.0f;
    pid_roll.inner_output_max = 1000.0f;
    pid_roll.inner_output_min = -1000.0f;
    pid_roll.outer_integral_max = 50.0f;
    pid_roll.inner_integral_max = 50.0f;

    // 初始化 pid_pitch（如果参数一样，可复制，也可改）
    pid_pitch = pid_roll;
/*
    // 初始化 pid_yaw（可单独设置不同参数）
    CascadePID_Init(&pid_yaw,
                    0.3f, 0.0f, 0.0f,   // outer
                    0.8f, 0.0f, 0.0f,   // inner
                    0.01f);            // sample_time
    pid_yaw.outer_output_max = 800.0f;
    pid_yaw.outer_output_min = -800.0f;
    pid_yaw.inner_output_max = 800.0f;
    pid_yaw.inner_output_min = -800.0f;
    pid_yaw.outer_integral_max = 40.0f;
    pid_yaw.inner_integral_max = 40.0f;
	*/
}


#define PPM_MIN 1000
#define PPM_MAX 2000
#define PWM_MIN 1000
#define PWM_MAX 2000
#define ANGLE_MIN -30.0f
#define ANGLE_MAX 30.0f

// 滑动平均滤波器的窗口大小
#define FILTER_WINDOW_SIZE 10

// 用于存储每个通道的历史PPM值
static uint16_t ppmHistory[12][FILTER_WINDOW_SIZE];
static uint8_t ppmIndex[12] = {0}; // 用于标记每个通道的当前索引

// 用于计算滑动平均值的函数ok
float applyMovingAverage(uint16_t* history) {
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
        sum += history[i];
    }
    return sum / (float)FILTER_WINDOW_SIZE;
}

// 将PPM值转换为角度的函数ok
float mapPPMToAngle(uint16_t ppmValue, uint16_t minPPM, uint16_t maxPPM, float minAngle, float maxAngle) {
    return minAngle + ((float)(ppmValue - minPPM) / (maxPPM - minPPM)) * (maxAngle - minAngle);
}

// 更新PPM历史值并计算滑动平均值ok
void updatePPMHistory(uint8_t channel, uint16_t ppmValue) {
    // 更新历史值
    ppmHistory[channel][ppmIndex[channel]] = ppmValue;
    
    // 更新索引，环形缓冲区
    ppmIndex[channel] = (ppmIndex[channel] + 1) % FILTER_WINDOW_SIZE;
}

// 将PPM值转换为姿态角，并通过参数传递ok
void calculateAttitudeAngles(float* pitch, float* roll, float* yaw) {
    // 获取每个通道的滑动平均值
    float avgPitch = applyMovingAverage(ppmHistory[1]);
    float avgRoll = applyMovingAverage(ppmHistory[3]);
    //float avgYaw = applyMovingAverage(ppmHistory[4]);

    // 将滑动平均值转换为角度
    *pitch = mapPPMToAngle(avgPitch, PPM_MIN, PPM_MAX, ANGLE_MIN, ANGLE_MAX);
    *roll = mapPPMToAngle(avgRoll, PPM_MIN, PPM_MAX, ANGLE_MIN, ANGLE_MAX);
    //*yaw = mapPPMToAngle(avgYaw, PPM_MIN, PPM_MAX, ANGLE_MIN, ANGLE_MAX);
	// 更新全局变量
    ppm_target_roll = *roll;
    ppm_target_pitch = *pitch;
    //ppm_target_yaw = *yaw;
    // 你可以在这里通过串口输出或者其他方式查看这些角度
    //Serial_SendFloat(*pitch);
	// 通过串口发送
	/*
    Serial_SendString("Pitch: ");
    Serial_SendFloat(*pitch);
    Serial_SendString("  Roll: ");
    Serial_SendFloat(*roll);
	*/
    //Serial_SendString("  Yaw: ");
    //Serial_SendFloat(*yaw);
    //Serial_SendString("\r\n");
	//Delay_ms(500);
	
}

// 示例：更新PPM值并计算姿态角ok
void updateAndCalculate(void)//升降是1，油门是2，副翼是3
{
    char buffer[16];

    for (int i = 1; i < 4; i++)  // 假设只处理通道 1~3
    {
        updatePPMHistory(i, ppmValues[i]);

        // 构造类似 "1500," 的字符串
        //sprintf(buffer, "%d,", ppmValues[i]);

        // 发送这个字符串
		/*
        for (int j = 0; j < strlen(buffer); j++)
        {
            USART_SendData(USART1, buffer[j]);
            while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        }
		*/
    }

    // 最后加换行，便于串口助手查看一帧
    //USART_SendData(USART1, '\n');
    //while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    // 计算姿态角
    calculateAttitudeAngles(&target_pitch, &target_roll, &target_yaw);
}


// 将 PID 输出映射到 PWM 控制量
uint16_t map_to_pwm(float pid_output, float output_min, float output_max)
{
    // 将 PID 输出映射到 PWM 范围
    float normalized_output = (pid_output - output_min) / (output_max - output_min);
    
    // 限制输出范围，防止溢出
    if (normalized_output > 1.0f) {
        normalized_output = 1.0f;
    } else if (normalized_output < 0.0f) {
        normalized_output = 0.0f;
    }
    
    // 映射到 PWM 范围
    uint16_t pwm_output = (uint16_t)(normalized_output * (PWM_MAX - PWM_MIN) + PWM_MIN);
    
    return pwm_output;
}

void control_loop(void)
{
	//calculateAttitudeAngles(&target_pitch, &target_roll, &target_yaw);
	updateAndCalculate();
    // 将 ppm 获取的各个方向的目标姿态角传给目标值
     target_roll = ppm_target_roll;
     target_pitch = ppm_target_pitch;
     //target_yaw = ppm_target_yaw;
	/*
	Serial_SendFloat(target_roll);
	Serial_SendFloat(target_pitch);
	Serial_SendString("\r\n");
	*/
	//Delay_ms(500);
	

    // 将传感器数据传递给外环反馈 (姿态角，单位为浮动值，可能需要除以 10)
	mpu6050_update();
    float outer_feedback_roll = rollint16 / 10.0f;
    float outer_feedback_pitch = pitchint16 / 10.0f;
    //float outer_feedback_yaw = yawint16 / 10.0f;
	
	//Serial_SendFloat(outer_feedback_roll);
	//Serial_SendString("\r\n");
	//Serial_SendFloat(outer_feedback_pitch);
	//Serial_SendString("\r\n");
	
	//Delay_ms(500);
	

    // 将角速度传递给内环反馈
	Mpu6050_GetValues();
    float inner_feedback_roll_rate = roll_rate;
    float inner_feedback_pitch_rate = pitch_rate;
    //float inner_feedback_yaw_rate = yaw_rate;
	
	Serial_SendFloat(inner_feedback_roll_rate);
	//Serial_SendFloat(inner_feedback_pitch_rate);
	Serial_SendString("\r\n");
	
	//Delay_ms(500);
	

    // 单独处理每个方向的 PID 控制
    float outer_output_roll = CascadePID_Calculate(&pid_roll, target_roll, outer_feedback_roll, inner_feedback_roll_rate);
    float outer_output_pitch = CascadePID_Calculate(&pid_pitch, target_pitch, outer_feedback_pitch, inner_feedback_pitch_rate);
	/*
	Serial_SendFloat(outer_output_roll);
	Serial_SendString("\r\n");
	Serial_SendFloat(outer_output_pitch);
	Serial_SendString("\r\n");
	Delay_ms(500);
	*/
    //float outer_output_yaw = CascadePID_Calculate(&pid_yaw, target_yaw, outer_feedback_yaw, inner_feedback_yaw_rate);

    // 将 PID 输出转换为 PWM 控制信号
    pwm_roll = map_to_pwm(outer_output_roll, pid_roll.outer_output_min, pid_roll.outer_output_max);
    pwm_pitch = map_to_pwm(outer_output_pitch, pid_pitch.outer_output_min, pid_pitch.outer_output_max);
    
	//uint16_t pwm_yaw = map_to_pwm(outer_output_yaw, pid_yaw.outer_output_min, pid_yaw.outer_output_max);
	/*
	Serial_SendFloat(pwm_roll);
	Serial_SendFloat(pwm_pitch);
	Serial_SendString("\r\n");
	*/
	//Delay_ms(500);
	/*
	pwm_roll=1500+(pwm_roll-1500)+(pwm_pitch-1500)+servoMiddlebias;
	pwm_pitch=1500+(pwm_roll-1500)-(pwm_pitch-1500)+servoMiddlebias;
	*/
	/*
	Serial_SendFloat(pwm_roll);
	Serial_SendFloat(pwm_pitch);
	Serial_SendString("\r\n");
	*/
	//Delay_ms(500);
	
    // 将 PWM 信号输出到硬件控制
    //servo_motor_speed_update();
}
