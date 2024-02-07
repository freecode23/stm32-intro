/*
 * it.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

#include "main_app.h"
extern TIM_HandleTypeDef htim3;

void SysTick_Handler (void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);
}


