/*
 * it.c
 *
 */

#include "main_app.h"
extern TIM_HandleTypeDef htimer6;
/**
 * SysTick, short for "System Tick Timer," is a basic, built-in timer in ARM Cortex-M microcontrollers, including STM32 series.
 * SysTick timer generates an interrupt at a fixed interval, defined by its configuration.
 * This interval is usually set to create a 1-millisecond time base,
 * but it can be adjusted according to the application's needs
 */
void SysTick_Handler (void)
{
	// Increment global tick variable.
	HAL_IncTick();


	HAL_SYSTICK_IRQHandler();
}


void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer6);
}
