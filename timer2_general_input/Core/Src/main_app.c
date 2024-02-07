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
void TIM2_Init(void);
void MC01_Configuration(void);

// UART helpers.
void UART2_Init(void);
void logUART(const char *format, ...);

UART_HandleTypeDef huart2;

// Timer 2 and captures variable.
TIM_HandleTypeDef htim2;
uint32_t input_captures[2] = { 0 };
uint8_t count = 1;
volatile uint8_t is_capture_done = FALSE;

int main(void) {
	uint32_t capture_difference = 0;
	double timer2_cnt_freq = 0;
	double timer2_cnt_period = 0;
	double user_signal_time_period = 0;
	double user_signal_freq = 0;

	HAL_Init();

	SystemClock_Config();

	// Initialize peripherals.
	UART2_Init();
	TIM2_Init();

	// Start timer2.
	HAL_TIM_Base_Start_IT(&htim2);

	logUART("\nBefore IC start");
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	logUART("\nWhile 1");
	while (1) {

		// Question 1: It never prints is capture done is 1 here. even though the 3 lines below this will print 1.
		logUART("\nWhile loop=%d", is_capture_done);
		if (is_capture_done) {

			logUART("\nWhile loop capture_is_done=%d", is_capture_done);
			if (input_captures[1] > input_captures[0])
				capture_difference = input_captures[1] - input_captures[0];
			else
				capture_difference = (4294967295 - input_captures[0])
						+ input_captures[1];

			logUART("\nPCLK1 freq=%d\r\n", HAL_RCC_GetPCLK1Freq()); // 16Hz

			timer2_cnt_freq = (HAL_RCC_GetPCLK1Freq() * 2)
					/ (htim2.Init.Prescaler + 1);
			logUART("timer2_cnt_freq=%d\r\n", timer2_cnt_freq);

			timer2_cnt_period = 1 / timer2_cnt_freq;
			logUART("timer2_cnt_period=%d\r\n", timer2_cnt_period);

			// Compute the period of MCO1 signal.
			user_signal_time_period = capture_difference * timer2_cnt_period;
			logUART("user_signal_time_period=%d\r\n", user_signal_time_period);

			user_signal_freq = 1 / user_signal_time_period;
			logUART("user_signal_freq=%d\r\n", user_signal_freq);

			is_capture_done = FALSE;
		}

	}
	return 0;
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
	// Question 2: Why is it when this is commented out, we still get callback from
	// TIM_IC, even though there is no signal from MCO1 source.
	// If I comment this out the while loop above never executes.
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
 * Callback when interrupt is triggered in input capture of timer2.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
//		logUART("\nActive value = %d\r\n", HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1));

		if (!is_capture_done) {
			if (count == 1) {
				input_captures[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
				count++;
				//			logUART("\ncount == 1; count++%d\r\n", count);

			} else if (count == 2) {
				input_captures[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
				count = 1;
				is_capture_done = FALSE;
				//			logUART("\ncount == 2; capture is done\r\n");
			}
		}
	}
}

void TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * Initialize UART for logging purpose.
 */
void UART2_Init(void) {

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
		logUART("ERROR!!");
	}

}

