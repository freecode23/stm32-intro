/*
 * it.c
 *
 */

#include "stm32f4xx_hal.h"

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
