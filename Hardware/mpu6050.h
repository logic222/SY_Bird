#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"                  // Device header
//#include "MyI2C.h"
#include "mpu6050_reg.h"
typedef struct
{
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}mpu6050_param_t;
/*
#define 0x00 ACC_SENS_2G   16384.0f  acc=raw/16834
#define 0x08 ACC_SENS_4G   8192.0f   acc=raw/8192
#define 0x10 ACC_SENS_8G   4096.0f
#define 0x18 ACC_SENS_16G  2048.0f

#define 0x00 GYRO_SENS_250 131.0f   gyro=raw/131
#define 0x08 GYRO_SENS_500 65.5f
#define 0x10 GYRO_SENS_1000 32.8f
#define 0x18 GYRO_SENS_2000 16.4f
*/
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
//void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length);
uint8_t MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Length);
uint8_t MPU6050_GetID(void);
void MPU6050_Init(void);
void Mpu6050_AccData(void);
void Mpu6050_GyroData(void);
void GyroOffset_Init(void);
void Mpu6050_GetValues(void);
float invSqrt(float x) ;
void get_eulerAngle(void);
void mpu6050_update(void);
void MPU6050_CalibrateGyro(uint16_t sampleCount);
void MahonyAHRSupdate(mpu6050_param_t *mpu6050);
#endif
