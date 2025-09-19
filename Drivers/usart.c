// PA9为串口调试
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

uint16_t uart1_test_data[] = {0xFE, 0x01};

void MyUsart_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);
    
    USART_Cmd(USART1, ENABLE);
    
    // 添加延时确保串口稳定
    //for (volatile int i = 0; i < 100000; i++);
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Array[i]);
    }
}

void Serial_SendTest(void)
{
    USART_SendData(USART1, 0x44);
}

// 发送一个字符
void Serial_SendChar(char ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint16_t)ch);
}

void Serial_SendString(char *str)
{
    while (*str)
    {
        Serial_SendChar(*str++);
    }
}

// 发送16位有符号整数
void Serial_SendInt(int16_t num)
{
    char buffer[10];
    sprintf(buffer, "%d", num);
    Serial_SendString(buffer);
}

// 发送32位有符号整数
void Serial_SendInt32(int32_t num)
{
    char buffer[15];
    sprintf(buffer, "%ld", num);
    Serial_SendString(buffer);
}

// 发送十六进制数（8位）
void Serial_SendHex(uint8_t hex)
{
    char buffer[5];
    sprintf(buffer, "0x%02X", hex);
    Serial_SendString(buffer);
}

// 发送十六进制数（16位）
void Serial_SendHex16(uint16_t hex)
{
    char buffer[7];
    sprintf(buffer, "0x%04X", hex);
    Serial_SendString(buffer);
}

// 发送十六进制数（32位）
void Serial_SendHex32(uint32_t hex)
{
    char buffer[11];
    sprintf(buffer, "0x%08lX", hex);
    Serial_SendString(buffer);
}

void Serial_SendFloat(float value)
{
    char buffer[32];
    sprintf(buffer, "%.4f", value);
    Serial_SendString(buffer);
}

// 格式化输出函数（类似printf）
void Serial_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial_SendString(buffer);
}
