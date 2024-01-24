/*
 * it.c
 *
 */

#include "stm32f4xx_hal.h"

void SysTick_Handler (void)
{
	// Increment global tick variable.
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
