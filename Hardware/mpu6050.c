//SCL PB10
//SDA PB11

#include "mpu6050.h"
#include "delay.h"
#include "math.h"
#include "string.h"
#include "usart.h"
#include <stdio.h>
#define MPU6050_Address 0xD0
#define PI          3.1415926f
#define dt          0.001f

uint8_t IIC_Data[7];
int16_t  mpu6050_acc_x,mpu6050_acc_y,mpu6050_acc_z;
int16_t mpu6050_gyro_x,mpu6050_gyro_y,mpu6050_gyro_z;
uint8_t sendflag=0;
int16_t rollint16 = 0;
int16_t pitchint16 = 0;
int16_t yawint16 = 0;
uint8_t send_attitude[11] = {0xFE, 0xEF, 0x0B, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
float roll,yaw,pitch;
float roll_rate,yaw_rate,pitch_rate;
/*
typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}mpu6050_param_t;
*/

typedef struct
{
	float yaw;
	float roll;
	float pitch;
}oula_param_t;

typedef struct 
{
	float GyroX;
	float GyroY;
	float GyroZ;
}gyro_param_t;

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
}q_param_t;

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 积分误差
float Ki = 0.0f, Kp = 5.0f; // PID参数
// 上一次调用时间
uint32_t lastUpdate = 0;
mpu6050_param_t mpu6050_data;
q_param_t  Q_info;
oula_param_t eulerAngle;
gyro_param_t GyroOffset_Data;
/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */

/*
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_Address);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(Data);				//发送要写入寄存器的数据
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_Stop();						//I2C终止
}
*/

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
  /*
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();						//I2C起始
	MyI2C_SendByte(MPU6050_Address);	//发送从机地址，读写位为0，表示即将写入
	MyI2C_ReceiveAck();					//接收应答
	MyI2C_SendByte(RegAddress);			//发送寄存器地址
	MyI2C_ReceiveAck();					//接收应答
	
	MyI2C_Start();						//I2C重复起始
	MyI2C_SendByte(MPU6050_Address | 0x01);	//发送从机地址，读写位为1，表示即将读取
	MyI2C_ReceiveAck();					//接收应答
	Data = MyI2C_ReceiveByte();			//接收指定寄存器的数据
	MyI2C_SendAck(1);					//发送应答，给从机非应答，终止从机的数据输出
	MyI2C_Stop();						//I2C终止
	
	return Data;
}
*/
/*
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length)
{
    uint8_t i;

    MyI2C_Start();                         // I2C起始
    MyI2C_SendByte(MPU6050_Address);        // 发送从机地址，读写位为0，表示写入操作
    MyI2C_ReceiveAck();                    // 接收应答
    MyI2C_SendByte(RegAddress);            // 发送寄存器地址
    MyI2C_ReceiveAck();                    // 接收应答
    
    MyI2C_Start();                         // I2C重复起始
    MyI2C_SendByte(MPU6050_Address | 0x01); // 发送从机地址，读写位为1，表示读取操作
    MyI2C_ReceiveAck();                    // 接收应答

    for (i = 0; i < Length; i++)
    {
        if (i == (Length - 1))  // 如果是最后一个字节
        {
            Data[i] = MyI2C_ReceiveByte();  // 接收字节
            MyI2C_SendAck(1);                // 发送非应答，终止数据输出
        }
        else
        {
            Data[i] = MyI2C_ReceiveByte();  // 接收字节
            MyI2C_SendAck(0);                // 发送应答，继续读取
        }
    }

    MyI2C_Stop();                          // I2C终止
}
*/



/**
  * 函    数：MPU6050等待事件
  * 参    数：同I2C_CheckEvent
  * 返 回 值：无
  */
void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									//给定超时计数时间
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	//循环等待指定事件
	{
		Timeout --;										//等待时，计数值自减
		if (Timeout == 0)								//自减到0后，等待超时
		{
			/*超时的错误处理代码，可以添加到此处*/
			break;										//跳出等待，不等了
		}
	}
}

/**
  * 函    数：MPU6050写寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 参    数：Data 要写入寄存器的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);										//硬件I2C生成起始条件
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_Address, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2C2, RegAddress);											//硬件I2C发送寄存器地址
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);			//等待EV8
	
	I2C_SendData(I2C2, Data);												//硬件I2C发送数据
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTOP(I2C2, ENABLE);											//硬件I2C生成终止条件
}

/**
  * 函    数：MPU6050读寄存器
  * 参    数：RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * 返 回 值：读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C2, ENABLE);										//硬件I2C生成起始条件
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_Address, I2C_Direction_Transmitter);	//硬件I2C发送从机地址，方向为发送
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);	//等待EV6
	
	I2C_SendData(I2C2, RegAddress);											//硬件I2C发送寄存器地址
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);				//等待EV8_2
	
	I2C_GenerateSTART(I2C2, ENABLE);										//硬件I2C生成重复起始条件
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);					//等待EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_Address, I2C_Direction_Receiver);		//硬件I2C发送从机地址，方向为接收
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);		//等待EV6
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);									//在接收最后一个字节之前提前将应答失能
	I2C_GenerateSTOP(I2C2, ENABLE);											//在接收最后一个字节之前提前申请停止条件
	
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);				//等待EV7
	Data = I2C_ReceiveData(I2C2);											//接收数据寄存器
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);									//将应答恢复为使能，为了不影响后续可能产生的读取多字节操作
	
	return Data;
}

uint8_t MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length) 
{
    // 1. 发送起始条件 + 设备地址（写模式）
    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);  // 等待事件发生，无需判断返回值

    I2C_Send7bitAddress(I2C2, MPU6050_Address, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);  // 等待事件发生，无需判断返回值

    // 2. 发送要读取的寄存器地址
    I2C_SendData(I2C2, RegAddress);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);  // 等待事件发生，无需判断返回值

    // 3. 重复起始条件 + 设备地址（读模式）
    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);  // 等待事件发生，无需判断返回值

    I2C_Send7bitAddress(I2C2, MPU6050_Address, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);  // 等待事件发生，无需判断返回值

    // 4. 连续读取多个字节
    for (uint8_t i = 0; i < Length; i++) {
        if (i == Length - 1) {
            // 最后一个字节：禁用ACK + 生成STOP
            I2C_AcknowledgeConfig(I2C2, DISABLE);
            I2C_GenerateSTOP(I2C2, ENABLE);
        }
        
        // 等待数据接收完成
        MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);  // 等待事件发生，无需判断返回值
        
        Data[i] = I2C_ReceiveData(I2C2);
    }

    // 恢复ACK使能（为后续操作做准备）
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return 1;
}


uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}
/*
void MPU6050_Init(void)
{
	MyI2C_Init();									//先初始化底层的I2C
	*/
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
/*
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
}
*/

void MPU6050_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);		//开启I2C2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB10和PB11引脚初始化为复用开漏输出
	
	/*I2C初始化*/
	I2C_InitTypeDef I2C_InitStructure;						//定义结构体变量
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;				//模式，选择为I2C模式
	I2C_InitStructure.I2C_ClockSpeed = 50000;				//时钟速度，选择为50KHz
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;		//时钟占空比，选择Tlow/Thigh = 2
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;				//应答，选择使能
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//应答地址，选择7位，从机模式下才有效
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;				//自身地址，从机模式下才有效
	I2C_Init(I2C2, &I2C_InitStructure);						//将结构体变量交给I2C_Init，配置I2C2
	
	/*I2C使能*/
	I2C_Cmd(I2C2, ENABLE);									//使能I2C2，开始运行
	
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);					//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);			//加速度计配置寄存器，选择满量程为±16g
}

void Mpu6050_AccData(void)
{
    uint8_t acc_data[6]; // 用来存储加速度计数据
    uint8_t reg = MPU6050_ACCEL_XOUT_H; // 从加速度计的高位寄存器开始读取

    // 读取6个字节的加速度计数据
    MPU6050_ReadRegs(reg, acc_data, 6);

    // 打印原始数据
    //Serial_SendByte(acc_data[0]);
    //Serial_SendByte(acc_data[1]);
    //Serial_SendByte(acc_data[2]);
    //Serial_SendByte(acc_data[3]);
    //Serial_SendByte(acc_data[4]);
    //Serial_SendByte(acc_data[5]);

    // 将读取到的数据拼接成16位有符号整数，表示X、Y、Z轴的加速度
    mpu6050_acc_x = (int16_t)((acc_data[0] << 8) | acc_data[1]);
    mpu6050_acc_y = (int16_t)(((uint16_t)acc_data[2] << 8) | acc_data[3]);
    mpu6050_acc_z = (int16_t)(((uint16_t)acc_data[4] << 8) | acc_data[5]);

    // 将加速度转换为g单位（假设量程为 ±16g）
    float acc_x_g = (float)mpu6050_acc_x / 2048.0f;
    float acc_y_g = (float)mpu6050_acc_y / 2048.0f;
    float acc_z_g = (float)mpu6050_acc_z / 2048.0f;

    // 发送加速度值到串口
	/*
    Serial_SendFloat(acc_x_g);
	Serial_SendString("\r\n");
	Delay_ms(100);
    Serial_SendFloat(acc_y_g);
	Serial_SendString("\r\n");
	Delay_ms(100);
    Serial_SendFloat(acc_z_g);
	Serial_SendString("\r\n");
	Delay_ms(100);
	*/
}




void Mpu6050_GyroData(void)
{
    uint8_t gyro_data[6]; // 用来存储陀螺仪数据
    uint8_t reg = MPU6050_GYRO_XOUT_H; // 从陀螺仪的高位寄存器开始读取

    // 读取6个字节的陀螺仪数据
    MPU6050_ReadRegs(reg, gyro_data, 6);

    // 打印原始数据，查看是否正常读取到数据
	/*
    Serial_SendByte(gyro_data[0]);
    Serial_SendByte(gyro_data[1]);
    Serial_SendByte(gyro_data[2]);
    Serial_SendByte(gyro_data[3]);
    Serial_SendByte(gyro_data[4]);
    Serial_SendByte(gyro_data[5]);
	*/

    // 解析数据为16位有符号整数
    mpu6050_gyro_x = (int16_t)((gyro_data[0] << 8) | gyro_data[1]);
    mpu6050_gyro_y = (int16_t)((gyro_data[2] << 8) | gyro_data[3]);
    mpu6050_gyro_z = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);

    // 将角速度转换为单位值
    float gyro_x_w = (float)mpu6050_gyro_x / 16.4f;
    float gyro_y_w = (float)mpu6050_gyro_y / 16.4f;
    float gyro_z_w = (float)mpu6050_gyro_z / 16.4f;

    // 发送陀螺仪角速度值到串口
	/*
	Serial_SendString("gyro_x:");
    Serial_SendFloat(gyro_x_w);
	Serial_SendString("\r\n");
	Delay_ms(500);
	Serial_SendString("gyro_y:");
    Serial_SendFloat(gyro_y_w);
	Serial_SendString("\r\n");
	Delay_ms(500);
	Serial_SendString("gyro_z:");
    Serial_SendFloat(gyro_z_w);
	Serial_SendString("\r\n");
	Delay_ms(500);
	*/
}


void GyroOffset_Init(void)
{
	
	GyroOffset_Data.GyroX=0;
	GyroOffset_Data.GyroY=0;
	GyroOffset_Data.GyroZ=0;
	for(uint16_t i=0;i<100;++i)
	{
		Mpu6050_GyroData();
		GyroOffset_Data.GyroX=GyroOffset_Data.GyroX+mpu6050_gyro_x;
		GyroOffset_Data.GyroY=GyroOffset_Data.GyroY+mpu6050_gyro_y;
		GyroOffset_Data.GyroZ=GyroOffset_Data.GyroZ+mpu6050_gyro_z;
		Delay_ms(1);
	}
	GyroOffset_Data.GyroX/=100;
	GyroOffset_Data.GyroY/=100;
	GyroOffset_Data.GyroZ/=100;
}


/**
  * @brief  陀螺仪自动校准
  * @param  sampleCount: 采样次数（建议 500~2000）
  */
void MPU6050_CalibrateGyro(uint16_t sampleCount)
{
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    int16_t gx, gy, gz;

    for(uint16_t i = 0; i < sampleCount; i++)
    {
        Mpu6050_GyroData();  // 读取原始陀螺仪数据 (int16_t)

        gx = mpu6050_gyro_x;
        gy = mpu6050_gyro_y;
        gz = mpu6050_gyro_z;

        sumX += gx;
        sumY += gy;
        sumZ += gz;

        Delay_ms(5); // 保证采样间隔，和你的数据更新频率一致
    }

    GyroOffset_Data.GyroX = (float)sumX / sampleCount;
    GyroOffset_Data.GyroY = (float)sumY / sampleCount;
    GyroOffset_Data.GyroZ = (float)sumZ / sampleCount;
}

void Mpu6050_GetValues(void)
{
    float alpha = 0.3;  // 滤波系数

    // 1. 调用Mpu6050_AccData来更新加速度数据
    Mpu6050_AccData();  // 获取最新的加速度数据
	Mpu6050_GyroData();


    // 2. 加速度数据滤波处理
    mpu6050_data.acc_x = (((float)mpu6050_acc_x) * alpha) / 2048 + mpu6050_data.acc_x * (1 - alpha);
    mpu6050_data.acc_y = (((float)mpu6050_acc_y) * alpha) / 2048 + mpu6050_data.acc_y * (1 - alpha);
    mpu6050_data.acc_z = (((float)mpu6050_acc_z) * alpha) / 2048 + mpu6050_data.acc_z * (1 - alpha);

    // 发送加速度数据
	/*
	Serial_SendString("acc_x");
    Serial_SendFloat(mpu6050_data.acc_x);
	Serial_SendString("\r\n");
	Delay_ms(500);
	Serial_SendString("acc_y");
    Serial_SendFloat(mpu6050_data.acc_y);
	Serial_SendString("\r\n");
	Delay_ms(500);
	Serial_SendString("acc_z");
    Serial_SendFloat(mpu6050_data.acc_z);
	Serial_SendString("\r\n");
	Delay_ms(500);
	*/

    // 3. 陀螺仪数据处理（并转化为角速度）
    mpu6050_data.gyro_x = ((float)mpu6050_gyro_x - GyroOffset_Data.GyroX) / 16.4f;
    mpu6050_data.gyro_y = ((float)mpu6050_gyro_y - GyroOffset_Data.GyroY) / 16.4f;
    mpu6050_data.gyro_z = ((float)mpu6050_gyro_z - GyroOffset_Data.GyroZ) / 16.4f;

    // 发送角速度数据
	
	Serial_SendString("gyro_x");
    Serial_SendFloat(mpu6050_data.gyro_x);
	//Serial_SendFloat(GyroOffset_Data.GyroX);
	Serial_SendString("\r\n");
	Delay_ms(500);
	Serial_SendString("gyro_y");
    Serial_SendFloat(mpu6050_data.gyro_y);
	//Serial_SendFloat(GyroOffset_Data.GyroY);
	Serial_SendString("\r\n");
	Delay_ms(500);
	Serial_SendString("gyro_z");
    Serial_SendFloat(mpu6050_data.gyro_z);
	//Serial_SendFloat(GyroOffset_Data.GyroZ);
	Serial_SendString("\r\n");
	Delay_ms(500);
	

    // 4. 角速度变化率（角度变化率）
    roll_rate = (((float)mpu6050_gyro_x - GyroOffset_Data.GyroX) / 16.4f) * 0.2f + roll_rate * 0.8f;
    pitch_rate = (((float)mpu6050_gyro_y - GyroOffset_Data.GyroY) / 16.4f) * 0.2f + pitch_rate * 0.8f;
    yaw_rate = (((float)mpu6050_gyro_z - GyroOffset_Data.GyroZ) / 16.4f) * 0.2f + yaw_rate * 0.8f;

    // 发送角度变化率数据
	/*
	Serial_SendString("r_rate");
    Serial_SendFloat(roll_rate);
	Serial_SendString("\r\n");
	Delay_ms(500);
	Serial_SendString("p_rate");
    Serial_SendFloat(pitch_rate);
	Serial_SendString("\r\n");
	Delay_ms(500);
	Serial_SendString("y_rate");
    Serial_SendFloat(yaw_rate);
	Serial_SendString("\r\n");
	Delay_ms(500);
	*/
}
/*
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5F3759DF - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y)); // 牛顿迭代
    return y;
}
*/

void MahonyAHRSupdate(mpu6050_param_t *mpu6050)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float gx, gy, gz;

    // ====== 固定 dt 测试 ======
    float deltaT = 0.001f;  // 1ms

    // ====== 读取 MPU6050 数据 ======
    Mpu6050_AccData();
    Mpu6050_GyroData();

    // 加速度转换 (单位 g, ±16g 模式下, LSB = 2048)
    mpu6050->acc_x = (float)mpu6050_acc_x / 2048.0f;
    mpu6050->acc_y = (float)mpu6050_acc_y / 2048.0f;
    mpu6050->acc_z = (float)mpu6050_acc_z / 2048.0f;

    // 陀螺仪转换 (rad/s, ±2000°/s, LSB = 16.4)
	
    gx = ((float)mpu6050_gyro_x - GyroOffset_Data.GyroX) / 16.4f * (PI / 180.0f);
    gy = ((float)mpu6050_gyro_y - GyroOffset_Data.GyroY) / 16.4f * (PI / 180.0f);
    gz = ((float)mpu6050_gyro_z - GyroOffset_Data.GyroZ) / 16.4f * (PI / 180.0f);


    // ====== 加速度归一化 ======
    recipNorm = invSqrt(mpu6050->acc_x * mpu6050->acc_x +
                        mpu6050->acc_y * mpu6050->acc_y +
                        mpu6050->acc_z * mpu6050->acc_z);
    mpu6050->acc_x *= recipNorm;
    mpu6050->acc_y *= recipNorm;
    mpu6050->acc_z *= recipNorm;

    // ====== 重力方向估计 ======
    // ====== 重力方向估计 (完整公式) ======
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

// ====== 误差 = 加速度与重力方向的叉积 ======
    halfex = (mpu6050->acc_y * vz - mpu6050->acc_z * vy);
    halfey = (mpu6050->acc_z * vx - mpu6050->acc_x * vz);
    halfez = (mpu6050->acc_x * vy - mpu6050->acc_y * vx);

// ====== 积分 + 比例反馈 ======
    if (Ki > 0.0f) {
        integralFBx += Ki * halfex * deltaT;
        integralFBy += Ki * halfey * deltaT;
        integralFBz += Ki * halfez * deltaT;
}   else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
}

    gx += Kp * halfex + integralFBx;
    gy += Kp * halfey + integralFBy;
    gz += Kp * halfez + integralFBz;


    // ====== 四元数更新 ======
    gx *= (0.5f * deltaT);
    gy *= (0.5f * deltaT);
    gz *= (0.5f * deltaT);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // ====== 四元数归一化 ======
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
	
	Serial_SendString("dt=");
    Serial_SendFloat(deltaT);
    Serial_SendString(" acc_norm=");
    Serial_SendFloat(sqrtf(mpu6050->acc_x*mpu6050->acc_x +
                           mpu6050->acc_y*mpu6050->acc_y +
                           mpu6050->acc_z*mpu6050->acc_z));
    Serial_SendString(" gx=");
    Serial_SendFloat(gx);
    Serial_SendString(" gy=");
    Serial_SendFloat(gy);
    Serial_SendString(" gz=");
    Serial_SendFloat(gz);
    Serial_SendString("\r\n");
	
	Serial_SendString("halfex=");
    Serial_SendFloat(halfex);
    Serial_SendString("halfey=");
    Serial_SendFloat(halfey);
    Serial_SendString("halfez=");
    Serial_SendFloat(halfez);
    Serial_SendString("\r\n");
	//Serial_SendString("q3=");
    //Serial_SendFloat(q3);
	//Serial_SendString("\r\n");
}
/*

void get_eulerAngle(void)
{
	MahonyAHRSupdate(&mpu6050_data);
	//Serial_SendFloat(Q_info.q0);
	
	float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    eulerAngle.pitch = asin(2 * q0 * q2 - 2 * q1 * q3) * 180 / PI; 
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI; 
    eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI;
	pitch = eulerAngle.pitch;
	roll = eulerAngle.roll;
	yaw = eulerAngle.yaw;
	
	Serial_SendString("roll:");
	Serial_SendFloat(roll);
	Serial_SendString("\r\n");
	Delay_ms(10);
	Serial_SendString("pitch:");
	Serial_SendFloat(pitch);
	Serial_SendString("\r\n");
	Delay_ms(10);
	Serial_SendString("yaw:");
	Serial_SendFloat(yaw);
	Serial_SendString("\r\n");
	Delay_ms(10);
	
	
	
}
*/

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

/**
  * @brief  Mahony AHRS 更新
  */
/*
void MahonyAHRSupdate(mpu6050_param_t *mpu6050)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float gx, gy, gz;

    // ====== 固定 dt 测试 ======
    float deltaT = 0.001f;  // 1ms，先用固定值调试，看漂移是否改善

    // ====== 读取 MPU6050 数据 ======
    Mpu6050_AccData();
    Mpu6050_GyroData();

    // 加速度转换 (单位 g, ±16g 模式下)
    mpu6050->acc_x = (float)mpu6050_acc_x / 2048.0f;
    mpu6050->acc_y = (float)mpu6050_acc_y / 2048.0f;
    mpu6050->acc_z = (float)mpu6050_acc_z / 2048.0f;

    // 陀螺仪转换 (rad/s, ±2000°/s)
	
    gx = ((float)mpu6050_gyro_x - GyroOffset_Data.GyroX) / 16.4f * (PI / 180.0f);
    gy = ((float)mpu6050_gyro_y - GyroOffset_Data.GyroY) / 16.4f * (PI / 180.0f);
    gz = ((float)mpu6050_gyro_z - GyroOffset_Data.GyroZ) / 16.4f * (PI / 180.0f);


    gx=0.0f;
	gy=0.0f;
	gz=0.0f;
	
    // ====== 调试输出 ======
    Serial_SendString("dt=");
    Serial_SendFloat(deltaT);
    Serial_SendString(" acc_norm=");
    Serial_SendFloat(sqrtf(mpu6050->acc_x*mpu6050->acc_x +
                           mpu6050->acc_y*mpu6050->acc_y +
                           mpu6050->acc_z*mpu6050->acc_z));
    Serial_SendString(" gx=");
    Serial_SendFloat(gx);
    Serial_SendString(" gy=");
    Serial_SendFloat(gy);
    Serial_SendString(" gz=");
    Serial_SendFloat(gz);
    Serial_SendString("\r\n");
	
	Serial_SendString("q0=");
    Serial_SendFloat(q0);
    Serial_SendString("q1=");
    Serial_SendFloat(q1);
    Serial_SendString("q2=");
    Serial_SendFloat(q2);
    Serial_SendString("\r\n");
	Serial_SendString("q3=");
    Serial_SendFloat(q3);
	Serial_SendString("\r\n");

    // ====== 加速度归一化 ======
    recipNorm = invSqrt(mpu6050->acc_x * mpu6050->acc_x +
                        mpu6050->acc_y * mpu6050->acc_y +
                        mpu6050->acc_z * mpu6050->acc_z);
    mpu6050->acc_x *= recipNorm;
    mpu6050->acc_y *= recipNorm;
    mpu6050->acc_z *= recipNorm;

    // ====== 重力方向估计 ======
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // ====== 误差（加速度和重力方向的差） ======
    halfex = (mpu6050->acc_y * halfvz - mpu6050->acc_z * halfvy);
    halfey = (mpu6050->acc_z * halfvx - mpu6050->acc_x * halfvz);
    halfez = (mpu6050->acc_x * halfvy - mpu6050->acc_y * halfvx);

    // 积分误差
    integralFBx -= Ki * halfex * deltaT;
    integralFBy -= Ki * halfey * deltaT;
    integralFBz -= Ki * halfez * deltaT;

    // 应用 PI 补偿
    gx -= Kp * halfex + integralFBx;
    gy -= Kp * halfey + integralFBy;
    gz -= Kp * halfez + integralFBz;

    // 四元数更新
    gx *= (0.5f * deltaT);
    gy *= (0.5f * deltaT);
    gz *= (0.5f * deltaT);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 四元数归一化
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
*/

/**
  * @brief  获取欧拉角
  */
void get_eulerAngle(void)
{
    MahonyAHRSupdate(&mpu6050_data);

    eulerAngle.pitch = asinf(2.0f * (q0 * q2 - q1 * q3)) * 180.0f / PI;
    eulerAngle.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                              1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    eulerAngle.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                              1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

    roll  = eulerAngle.roll;
    pitch = eulerAngle.pitch;
    yaw   = eulerAngle.yaw;
	
	Serial_SendString("roll:");
	Serial_SendFloat(roll);
	Serial_SendString("\r\n");
	Delay_ms(10);
	Serial_SendString("pitch:");
	Serial_SendFloat(pitch);
	Serial_SendString("\r\n");
	Delay_ms(10);
	Serial_SendString("yaw:");
	Serial_SendFloat(yaw);
	Serial_SendString("\r\n");
	Delay_ms(10);
}
/*
void mpu6050_update(void)
{
	get_eulerAngle();
	
	sendflag++;
	if(sendflag > 100)
	{
		sendflag = 0;
		
		rollint16 = eulerAngle.roll*10;
		pitchint16 = eulerAngle.pitch*10;
		yawint16 = eulerAngle.yaw*10;
		//Serial_SendFloat(rollint16);
		memcpy(send_attitude+4,&rollint16,sizeof(int16_t));
		memcpy(send_attitude+6,&pitchint16,sizeof(int16_t));
		memcpy(send_attitude+8,&yawint16,sizeof(int16_t));
		send_attitude[10] = 0;
		for(int i = 0; i < 10; i++)
		{
			send_attitude[10] = send_attitude[10]+send_attitude[i];
		}
		
		Serial_SendArray(send_attitude,11);
	}
}
*/


void mpu6050_update(void) {
    get_eulerAngle();  // 更新欧拉角数据

    sendflag++;
    if (sendflag > 10)
    {
        sendflag = 0;

        // 角度转换为整数，保留1位小数（乘以10）
        rollint16 = (int16_t)(eulerAngle.roll * 10);
        pitchint16 = (int16_t)(eulerAngle.pitch * 10);
        yawint16 = (int16_t)(eulerAngle.yaw * 10);

        char buffer[64];
        
        // 加上表头 "roll,pitch,yaw"
        //sprintf(buffer, "roll,pitch,yaw:%d,%d,%d\n", rollint16, pitchint16, yawint16);

        // 逐字节发送
		/*
        for (int i = 0; i < strlen(buffer); i++)
        {
            USART_SendData(USART1, buffer[i]);
            while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        }
		*/
		
    }
		
	}



