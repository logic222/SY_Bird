//PA1:左舵，PA2:电调，PA3：右舵
#include "servo_motor.h"
#include "ppm.h"
#include "flightcontrol_b.h"
#include <string.h>
#include "scheduler.h"
#include "usart.h"
uint16_t motor_speed;
uint16_t servol_speed;//左舵
uint16_t servor_speed;//右舵
extern uint16_t pwm_roll;
extern uint16_t pwm_pitch;
extern volatile uint16_t ppmValues[12];
uint16_t ch2;
uint16_t ch4;


// 舵机参数定义
#define SERVO_MIN 600   // 1ms 最小角度
#define SERVO_MID 1800   // 1.5ms 中位
#define SERVO_MAX 2800   // 2ms 最大角度
#define STEP_SIZE 10     // 每次步进值
#define DELAY_MS 50      // 步进间隔时间(ms)

/*
void TIM2_PWM_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	// 添加AFIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; // PA1,PA2,PA3
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;    // 高速
    GPIO_Init(GPIOA, &GPIO_InitStruct);              // 初始化GPIOA
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period=20000-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler=168-1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;							//定义结构体变量
	TIM_OCStructInit(&TIM_OCInitStructure);                         //结构体初始化，若结构体没有完整赋值
	                                                                //则最好执行此函数，给结构体所有成员都赋一个默认值
	                                                                //避免结构体初值不确定的问题
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //输出比较模式，选择PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;       //输出极性，选择为高，若选择极性为低，则输出高低电平取反
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //输出使能
	TIM_OCInitStructure.TIM_Pulse = 1500;								//初始的CCR值
	// 通道2
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

// 通道3
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

// 通道4
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	// --- 在启动定时器之前，先设置好舵机为中位 ---
    TIM_SetCompare2(TIM2, 1500); // 左舵
    TIM_SetCompare3(TIM2, 1000); // 电调
    TIM_SetCompare4(TIM2, 1500); // 右舵
	*/	/*TIM使能*/
	//TIM_Cmd(TIM2, ENABLE);			//使能TIM2，定时器开始运行
	//TIM_SetCompare2(TIM2, 1500); // 直接初始化到安全位置
    //TIM_SetCompare3(TIM2, 1000);
    //TIM_SetCompare4(TIM2, 1500);
//}

void TIM2_PWM_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; // TIM2_CH2, CH3, CH4
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseInitTypeDef tim;
    tim.TIM_Period = 20000 - 1;              // 20ms
    tim.TIM_Prescaler = 72 - 1;              // 72MHz / 72 = 1MHz（1us单位）
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_OCInitTypeDef pwm;
    pwm.TIM_OCMode = TIM_OCMode_PWM1;
    pwm.TIM_OutputState = TIM_OutputState_Enable;
    pwm.TIM_OCPolarity = TIM_OCPolarity_High;

    pwm.TIM_Pulse = 1500;  // 中立值
    TIM_OC2Init(TIM2, &pwm);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    pwm.TIM_Pulse = 1000;  // 电调低速空转
    TIM_OC3Init(TIM2, &pwm);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    pwm.TIM_Pulse = 1500;  // 中立值
    TIM_OC4Init(TIM2, &pwm);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);//重要
    TIM_Cmd(TIM2, ENABLE);
}

// 打印当前PWM值到串口
void Print_PWM_Values(void) {
    char buffer[128];
    sprintf(buffer, 
        "L:%d R:%d Motor:%d\n",
        TIM_GetCapture2(TIM2),  // 左舵机PA1 (TIM2_CH2)
        TIM_GetCapture4(TIM2),  // 右舵机PA3 (TIM2_CH4)
        TIM_GetCapture3(TIM2)   // 电调PA2 (TIM2_CH3)
    );
    
    for (int i = 0; i < strlen(buffer); i++) {
        USART_SendData(USART1, buffer[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

void servo_motor_init(void)
{
	//Delay_ms(50);
    //TIM_SetCompare2(TIM2, 1800); // 直接初始化到安全位置
    //TIM_SetCompare3(TIM2, 1000);
    //TIM_SetCompare4(TIM2, 1800);
    //Print_PWM_Values(); // 打印初始化后的PWM值
    //motor_speed = 1000;
    //servol_speed = 1500;
    //servor_speed = 1500;

}
/*
void servo_motor_speed_update(void)
{
	motor_speed=ppmValues[2];
	//servol_speed=ppmValues[1];//左舵
	//servor_speed=ppmValues[3];//右舵
	// 示例：将 1200~1800 映射到 800~2400
	// 当前信号范围：1099~1938μs → 映射到800~2400μs
    //servol_speed = 2400 - (ppmValues[1] - 1099) * (2400 - 800) / (1938 - 1099);
	servol_speed = 2600 - (ppmValues[1] - 1099) * (2600 - 600) / (1928 - 1099);
    servor_speed = 2600 - (ppmValues[3] - 1099) * (2600 - 600) / (1938 - 1099);
	//servol_speed=pwm_roll;
	//servor_speed=pwm_pitch;
	TIM_SetCompare2(TIM2,servol_speed);
	TIM_SetCompare3(TIM2,motor_speed);
	TIM_SetCompare4(TIM2,servor_speed);
}
*/
/*
void servo_motor_speed_update(void)
{
    // 1. 获取各通道原始值
    uint16_t ch2 = ppmValues[1]; // 2通道 - 升降
    uint16_t ch4 = ppmValues[3]; // 1通道 - 副翼
    
    // 2. 电调控制保持不变
    motor_speed = ppmValues[2];
    
    // 3. 计算升降控制量（同向）
    // 将1099-1938映射到-500~500范围
    int elevator = map(ch2, 1099, 1938, -500, 500);
    
    // 4. 计算方向控制量（差动）
    // 将1099-1938映射到-500~500范围
    int rudder = map(ch4, 974, 1809, -500, 500);
    
    // 5. 混合控制
    // 左舵机：升降+方向
    servol_speed = 1500 + elevator + rudder;
    // 右舵机：升降-方向
    servor_speed = 1500 + elevator - rudder;
    
    // 6. 限制输出范围
    servol_speed = constrain(servol_speed, 600, 2400);
    servor_speed = constrain(servor_speed, 600, 2400);
    
    // 7. 更新PWM输出
    TIM_SetCompare2(TIM2, servol_speed); // PA1: 左舵
    TIM_SetCompare3(TIM2, motor_speed);  // PA2: 电调
    TIM_SetCompare4(TIM2, servor_speed); // PA3: 右舵
}
*/
void servo_motor_speed_update(void)
{
    static enum {INIT, WAIT, ACTIVE} state = INIT;
    static uint32_t init_time = 0;

    switch (state) {
        case INIT:
            // 设置为中立位置
            TIM_SetCompare2(TIM2, 1500); // 左舵
            TIM_SetCompare3(TIM2, 1000); // 电调空转（安全）
            TIM_SetCompare4(TIM2, 1500); // 右舵
            init_time = GetTick();       // 记录时间
            state = WAIT;
            break;

        case WAIT:
            if (GetTick() - init_time >= 1000) // 等待1秒
                state = ACTIVE;
            break;

        case ACTIVE:
            // 以下是你原本的混控逻辑
             ch2 = ppmValues[1]; // 升降
             ch4 = ppmValues[3]; // 副翼
            motor_speed = ppmValues[2];

            int elevator = map(ch2, 1099, 1938, -500, 500);
            int rudder = map(ch4, 974, 1809, -500, 500);
/*
            servol_speed = 1500 + elevator + rudder;
            servor_speed = 1500 + elevator - rudder;
			
            servol_speed = constrain(servol_speed, 600, 2400);
            servor_speed = constrain(servor_speed, 600, 2400);
		*/
			

			control_loop();
		/*
			Serial_SendFloat(pwm_roll);
			Serial_SendFloat(pwm_pitch);
			Serial_SendString("\r\n");
		*/
			
			int pwm_left  = 1500 + (pwm_pitch-1500) + (pwm_roll-1500); // pitch + roll
			int pwm_right = 1500 + (pwm_pitch-1500) - (pwm_roll-1500); // pitch - roll
			
			servol_speed = elevator+pwm_left;
			servor_speed = rudder+pwm_right;
			servol_speed = constrain(servol_speed, 1000, 2000);   // 限制范围
			servor_speed = constrain(servor_speed,1000, 2000);
			
			/*
			Serial_SendString("servol_speed: ");
			Serial_SendFloat(servol_speed);
			Serial_SendString(", servor_speed: ");
			Serial_SendFloat(servor_speed);
			Serial_SendString("\r\n");
*/
            TIM_SetCompare2(TIM2, servol_speed);
            TIM_SetCompare3(TIM2, motor_speed);
            TIM_SetCompare4(TIM2, servor_speed);
            break;
    }
}

// 映射函数（如果尚未定义）
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 限制函数（如果尚未定义）
int constrain(int value, int min_val, int max_val) {
    return value < min_val ? min_val : (value > max_val ? max_val : value);
}
/*
void Servo_Sweep(void) {
    static uint16_t current_pos = SERVO_MID;
    static int8_t direction = 1; // 1:增加, -1:减小
    
    // 更新位置
    current_pos += (STEP_SIZE * direction);
    
    // 边界检查
    if(current_pos >= SERVO_MAX) {
        current_pos = SERVO_MAX;
        direction = -1;
    } 
    else if(current_pos <= SERVO_MIN) {
        current_pos = SERVO_MIN;
        direction = 1;
    }
    
    // 设置新位置
    TIM_SetCompare2(TIM2, current_pos);
}
*/
