/*
 * main_app.c
 *
 *  Created on: Jan 23, 2024
 *      Author: sherlyhartono
 *
 *
 * Make sure to connect PA0 which is the pin for Timer 2 ch1 input capture
 * to PA8 which is the clock signal of MCO1 before running the program.
 *
 * During rising edge on timer2 ch1, the timer captures
 * the current value of its counter register (CNT)
 * and stores it in the capture/compare register (e.g., CCR1 for channel 1).
 */
#include "main_app.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h> // Include for va_list and related functions

void Error_Handler(void);
void SystemClock_Config(void);
void TIMER2_Init(void);

// UART helpers.
void UART2_Init(void);
void logUART(const char *format, ...);
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
volatile uint32_t ccr_content;

volatile uint32_t pulse1_value = 25000; //to produce 500Hz

int main(void) {

	HAL_Init();

	SystemClock_Config();
	// Initialize peripherals.
	UART2_Init();

	TIMER2_Init();
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		logUART("Error initializing Base!!\r\n");
		Error_Handler();
	}
	logUART("Finish init Base=\r\n");
	if (HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
		logUART("Error initializing OC!!\r\n");
		Error_Handler();
	}
	logUART("Finish init OC=\r\n");
	while (1) {

	}

	return 0;
}

/**
 * Callback when the the CCR register matches the pulse value.
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	logUART("Callback=\r\n");
	/* TIM2_CH1 toggling with frequency = 500 Hz */
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		ccr_content = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		logUART("crrContent=%d\r\n", ccr_content);

		// Set the output of the GPIO as the increment of ccr value + pulse value.
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, ccr_content + pulse1_value);
	}
}

/**
 * Initialize timer2 Channel 1. for Output Compare at Pin PA15.
 */
void TIMER2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * Initialize UART for logging purpose.
 */
void UART2_Init(void) {

	// Step 1. Initialize the high level peripheral initialization.
	// Initialize the registers to the base address in the handler.
	// Get this from stm32f407xx.h
	huart2.Instance = USART2;

	// Parameter initialization i.e. baud rate, word length, stop bits, parity, etc.
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Instance = USART2;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;

	// Disabled means transmission and reception proceed without check if receiver is ready
	// before sending data.
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	// This will call the HAL_UART_MspInit(huart) low level hardware.
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * Init RCC clock.
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 50;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
 * Logging to UART to help debug.
 */
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
	}

}

