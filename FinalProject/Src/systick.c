/*
 * systick.c
 *
 *  Created on: 14. nov. 2022
 *      Author: kaspernyhus
 */


#include "systick.h"
#include "stm32f30x_conf.h"

volatile uint8_t systick = 0;

/**
 * @brief Enables TIMER2 as systick timer
 *
 */
void systick_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM2_InitStructure;
	TIM_TimeBaseStructInit(&TIM2_InitStructure);
	TIM2_InitStructure.TIM_Period = 2000 - 1;
	TIM2_InitStructure.TIM_Prescaler = 42000 - 1;
	TIM_TimeBaseInit(TIM2, &TIM2_InitStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM2, ENABLE);
}


void TIM2_IRQHandler(void)
{
	systick = 1;
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
