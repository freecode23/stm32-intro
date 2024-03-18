/*
 * main_app.c
 *
 *  Created on: Jan 23, 2024
 *      Author: sherlyhartono
 */


#include "stm32f4xx_hal.h"
#include "main_app.h"
#include <string.h>

void SystemClockConfig(void);
void Error_Handler(void);
void TIMER6_Init(void);
void GPIO_Init(void);

TIM_HandleTypeDef htimer6;

int main(void)
{
	HAL_Init();
	SystemClockConfig();

	// Initialize peripherals.
	GPIO_Init();
	TIMER6_Init();


	// Start the base timer.
	// THhis will enable the counter through TIM_CR1_CEN_Msk
	HAL_TIM_Base_Start(&htimer6);

	while(1)
	{
		// Loop until update event happens which set the timer 6's status register to 1.
		// TIM_SR_UIF just means 1 << 0;
		// UIF stands for Update Interrupt Flag
		while( ! (TIM6->SR & TIM_SR_UIF) );

		//  The required time delay has been elapsed.
		// Reset the flag.
		TIM6->SR = 0;

		// Toggle the LED.
		HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
	}
	return 0;
}

void SystemClockConfig(void)
{
	/**
	 * Use internal oscillator if this function is empty.
	 */

}


/**
 * initialize the timer peripheral.
 */
void TIMER6_Init(void)
{
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 249;
	// Source is 16Mhz
	// The number of ticks (stored in ARR).
	// We should subtract by 1 because the event is actually tirggered on
	// period + 1 tick. Which means at 64001 tick.
	// Which gives the total period of 64001 tick. That's why we should subtract by 1
	// from the number we compute.
	htimer6.Init.Period = 64000-1;


	// Init the HAL Timer.
	if( HAL_TIM_Base_Init(&htimer6) != HAL_OK )
	{
		Error_Handler();
	}
}

/**
 * Initilize GPIO to toggle the LED pin.
 * LED is driven by GPIO.
 */

void GPIO_Init(void)
{
	// Enable clock.
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// Initial output state is 0.
	HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);


	// The base address of the GPIO.
	GPIO_InitTypeDef GPIO_InitStruct;

	// LED Blue is shown as PD15 in ioc file.
	// This means its on port D pin 15.
	GPIO_InitStruct.Pin = LD6_Pin;

	// Use push pull so we don't need pull up resistor externally.
	// Push means drive high, pull means drive it low.
	// So micro-controller can turn it on and off directly without external resistor.
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	// Initialize GPIO at port D.
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void Error_Handler(void) {
	while (1) {
		/**
		 * maybe blink LED if there are errors.
		 */
	}

}

