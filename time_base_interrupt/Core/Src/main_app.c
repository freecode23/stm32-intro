/*
 * main_app.c
 *
 *  Created on: Jan 23, 2024
 *      Author: sherlyhartono
 */

#include "main_app.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h> // Add this line at the top of your file


void SystemClockConfig(void);
void Error_Handler(void);
void TIMER6_Init(void);
void GPIO_Init(void);
void logUART(const char *format, ...);
void MX_USART2_UART_Init();
TIM_HandleTypeDef htimer6;
UART_HandleTypeDef huart2;

int main(void) {
	HAL_Init();
	SystemClockConfig();

	// Initialize peripherals.
	GPIO_Init();
	TIMER6_Init();
	MX_USART2_UART_Init();

	// Start the base timer.
	// This will enable the counter through TIM_CR1_CEN_Msk.
	HAL_TIM_Base_Start_IT(&htimer6);
	logUART("PCKL1Freq %d Hz\r\n", HAL_RCC_GetPCLK1Freq());
	while (1)
		;

	return 0;
}

/**
 * initialize the timer peripheral.
 */
void TIMER6_Init(void) {
	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler = 249;

	// The number of ticks (stored in ARR).
	// We should subtract by 1 because the event is actually triggered on
	// period + 1 tick. Which means at 64001 tick.
	// Which gives the total period of 64001 tick. That's why we should subtract by 1
	// from the number we compute.
	// this update event triggers an interrupt.
	htimer6.Init.Period = 64000 - 1;

	// Init the HAL Timer.
	if (HAL_TIM_Base_Init(&htimer6) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * Initilize GPIO to toggle the LED pin.
 * LED is driven by GPIO.
 */

void GPIO_Init(void) {
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

	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
}

void SystemClockConfig(void) {
	/**
	 * Use internal oscillator if this function is empty.
	 */

}

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void logUART(const char *format, ...) {
	char usr_msg[100]; // Buffer for the formatted message
	va_list args;

	// Initialize the va_list variable with the last known fixed parameter (in this case, 'format')
	va_start(args, format);

	// Use vsnprintf to safely format the string into usr_msg
	// vsnprintf returns the number of characters that would have been written if usr_msg had been sufficiently large
	vsnprintf(usr_msg, sizeof(usr_msg), format, args);

	// Clean up the va_list when we're done with it
	va_end(args);

	// Transmit the formatted string via UART
	HAL_UART_Transmit(&huart2, (uint8_t*) usr_msg, strlen(usr_msg),
	HAL_MAX_DELAY);
}

void Error_Handler(void) {
	while (1) {
		/**
		 * maybe blink LED if there are errors.
		 */
	}

}

