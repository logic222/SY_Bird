#include "delay.h"

static __IO uint32_t TimingDelay;

void Delay_Init(void)
{
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1); // 配置失败
    }
}
/*
void SysTick_Handler(void)
{
    if (TimingDelay != 0)
        TimingDelay--;
}
*/
void Delay_ms(uint32_t ms)
{
    TimingDelay = ms;
    while (TimingDelay != 0);
}
