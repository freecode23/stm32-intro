/*
 * it.c
 *
 */

#include "main_app.h"
extern UART_HandleTypeDef huart2;

void SysTick_Handler (void)
{
	// Increment global tick variable.
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
