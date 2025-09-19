#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"                  // Device header

void MyUsart_Init(void);


void Serial_SendByte(uint8_t Byte);

void Serial_SendArray(uint8_t *Array,uint16_t Length);

void Serial_SendTest(void);
void Serial_SendChar(char ch);
void Serial_SendString(char *str);
void Serial_SendInt(int16_t num);
void Serial_SendFloat(float value);
void Serial_SendInt32(int32_t num);
void Serial_SendHex16(uint16_t hex);
void Serial_SendHex32(uint32_t hex);
void Serial_SendHex(uint8_t hex);
void Serial_Printf(const char *format, ...);
#endif 
