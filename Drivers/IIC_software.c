//软件模拟I2C信号（对中断敏感，此项目不能用）
#include "IIC_software.h"
#define II2_GPIO GPIOB
#define SCL_Pin GPIO_Pin_10
#define SDA_Pin GPIO_Pin_11

void SCL_W(uint8_t BitValue)
{
	GPIO_WriteBit(II2_GPIO,SCL_Pin,(BitAction)BitValue);
}

void SDA_W(uint8_t BitValue)
{
	GPIO_WriteBit(II2_GPIO,SDA_Pin,(BitAction)BitValue);
}

uint8_t SDA_R(void)
{
	uint8_t BitValue;
	BitValue=GPIO_ReadInputDataBit(II2_GPIO,SDA_Pin);
	return BitValue;
}

void IIC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin=SCL_Pin|SDA_Pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(II2_GPIO,&GPIO_InitStructure);
	
	GPIO_SetBits(II2_GPIO,SCL_Pin|SDA_Pin);
}

void IIC_Start(void)
{
	SDA_W(1);
	SCL_W(1);
	SDA_W(0);
	SCL_W(0);
}

void IIC_Stop(void)
{
	SCL_W(0);
	SDA_W(0);
	SCL_W(1);
	SDA_W(1);
}

void IIC_SendByte(uint8_t Byte)
{
	for(uint8_t i;i<8;i++)
	{
		SCL_W(0);
	    if((Byte & (0x80 >>i))==0)
	    {
		    SDA_W(0);
	    }
	    else
		    SDA_W(1);
	    SCL_W(1);
	    SCL_W(0);
	}
}

uint8_t IIC_ReceiveByte(void)
{
	uint8_t i,Byte=0x00;
	SDA_W(1);
	for(i=0;i<8;i++)
	{
		SCL_W(1);
	    if (SDA_R()==1)
		{
			Byte|=(0x80>>i);
		}
		SCL_W(0);
	}
	
	return Byte;
}

/**
  * @brief  通过I2C接收多个字节
  * @param  pData: 存储数据的缓冲区指针
  * @param  size:  要接收的字节数
  * @retval 无
  */
void IIC_ReceiveMultiBytes(uint8_t *pData, uint16_t size)
{
    while (size--)
    {
        *pData++ = IIC_ReceiveByte();  // 逐字节接收并存储
    }
}

void IIC_SendAck(uint8_t AckBit)
{
	
		SCL_W(0);
	    if((AckBit & 0x80)==0)
	    {
		    SDA_W(0);
	    }
	    else
		    SDA_W(1);
	    SCL_W(1);
	    SCL_W(0);
}

uint8_t IIC_ReceiveAck(void)
{
	uint8_t AckBit;
	SDA_W(1);
	
	SCL_W(1);
	if (SDA_R()==1)
	{
		AckBit|=0x80;
	}
	SCL_W(0);
	return AckBit;
}
