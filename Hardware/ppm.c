//接在PA6
#include "ppm.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

volatile uint16_t ppmValues[12];
volatile uint8_t ppmChannel = 0;
volatile uint32_t lastCapture = 0;

void PPM_Input_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 71; // 1us per tick
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE); // ✅ 关键：启用输入捕获中断（仅为了使能标志位）

	// TIM3 NVIC配置
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


    TIM_Cmd(TIM3, ENABLE);
}

void ppm_update(void)
{
    if (TIM_GetFlagStatus(TIM3, TIM_FLAG_CC1) == SET)
    {
        uint32_t currentCapture = TIM_GetCapture1(TIM3);
        uint32_t pulseWidth;

        if (currentCapture >= lastCapture)
            pulseWidth = currentCapture - lastCapture;
        else
            pulseWidth = (0xFFFF - lastCapture) + currentCapture;
/*
        char buffer[64];
        sprintf(buffer, "Capture: %u, Pulse: %u\n", currentCapture, pulseWidth);
        for (int i = 0; i < strlen(buffer); i++)
        {
            USART_SendData(USART1, buffer[i]);
            while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        }
*/
        if (pulseWidth > 2600)
        {
            ppmChannel = 0;
        }
        else if (pulseWidth >= 600 && pulseWidth <= 2500)
        {
            if (ppmChannel < 12)
            {
                ppmValues[ppmChannel++] = pulseWidth;
            }
        }

        lastCapture = currentCapture;
        TIM_ClearFlag(TIM3, TIM_FLAG_CC1);
    }
}
/*方向舵（4通道）是ppmValues[0]，升降舵（2通道）是ppmValues[1]，油门是ppmValues[2]，副翼（1通道）是ppmValues[3]*/
void ppm_send(void)
{
    char buffer[128];
    sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d\n",
            ppmValues[0], ppmValues[1], ppmValues[2], ppmValues[3],
            ppmValues[4], ppmValues[5], ppmValues[6], ppmValues[7]);

    for (int i = 0; i < strlen(buffer); i++)
    {
        USART_SendData(USART1, buffer[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {
        ppm_update();
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); // ✅ 清除中断标志
    }
}

