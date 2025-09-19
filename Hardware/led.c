//PB0为LED口
#include "led.h"
#include "delay.h"
extern uint8_t led_state;
void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
}

void led_blink(void)
{
	led_state = (led_state == Bit_SET) ? Bit_RESET : Bit_SET;
    GPIO_WriteBit(GPIOB, GPIO_Pin_0, led_state);
}
