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
 * During rising edge on timer3 ch1, the timer captures
 * the current value of its counter register (CNT)
 * and stores it in the capture/compare register (e.g., CCR1 for channel 1).
 */
#include "main_app.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h> // Include for va_list and related functions

void Error_Handler(void);
void SystemClock_Config(void);
void TIM3_Init(void);
static void MX_GPIO_Init(void);

// UART helpers.
void UART2_Init(void);
void logUART(const char *format, ...);

UART_HandleTypeDef huart2;

// Timer 2 and captures variable.
TIM_HandleTypeDef htim3;
volatile uint32_t input_captures[2] = { 0 };
uint8_t count = 1;
volatile uint8_t is_capture_done = FALSE;

int main(void) {
	uint32_t capture_difference = 0;
	double timer3_cnt_freq = 0;
	double timer3_cnt_period = 0;
	double user_signal_time_period = 0;
	double user_signal_freq = 0;

	HAL_Init();

	SystemClock_Config();

	// Initialize peripherals.
	MX_GPIO_Init();
	UART2_Init();
	TIM3_Init();

	// Start Timer3.
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		logUART("Error initializing Base timer \r\n");
		Error_Handler();
	}
	if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK) {
		logUART("Error initializing Gen Purpose timer with Input Capture \r\n");
		Error_Handler();
	}
	logUART("\nbefore while loop\r\n");
	while (1) {
		if (is_capture_done) {

			if (input_captures[1] > input_captures[0])
				capture_difference = input_captures[1] - input_captures[0];
			else
				capture_difference = (65535 - input_captures[0])
						+ input_captures[1];

//			logUART("\nPCLK1 freq=%d\r\n", HAL_RCC_GetPCLK1Freq());
//			logUART("input_captures[1]=%d\r\n", input_captures[1]);

			timer3_cnt_freq = (HAL_RCC_GetPCLK1Freq() * 2)
					/ (htim3.Init.Prescaler + 1);
//			logUART("\ntimer3_cnt_freq=%d\r\n", timer3_cnt_freq);

			timer3_cnt_period = 1 / timer3_cnt_freq;
//			logUART("timer3_cnt_period=%d\r\n", timer3_cnt_period);

			// Compute the period of MCO1 signal.
			user_signal_time_period = capture_difference * timer3_cnt_period;
//			logUART("user_signal_time_period=%d\r\n", user_signal_time_period);

			user_signal_freq = 1 / user_signal_time_period;
//			logUART("user_signal_freq=%d\r\n", user_signal_freq);

			is_capture_done = FALSE;
		}
	}
	return 0;
}

/**
 * Callback when interrupt is triggered in input capture of timer3.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	logUART("callback capture value=%d\r\n", __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1));

	if (!is_capture_done) {
		if (count == 1) {
			input_captures[0] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			count++;

		} else if (count == 2) {
			input_captures[1] = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
			count = 1;
			is_capture_done = TRUE;
		}
	}
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  // Bug:
  // If the source clock for MC01 is the same as SYSCLKSource interrupt will not be
  // triggered and the timer callback is never called.
  // MC0 have to be set to source from a different clock than SYCLKSource.
  // This will trigger the interrupt correctly, but we never enter the while loop in the main function, i.e.
  // the interrupt for timer is called infinitely before we even enter the while loop.
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}


/**
 * Initialize timer 3 for input capture at PIN PC6.
 */
void TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	// Step 1. Initialize the timer with `Input Capture` mode.
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;

	// Let it count up to the maximum value.
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	// Initialize the base and IC.
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	// Step 2. Configure the input capture channel.
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	// Select input Direct Capture:
	// This configuration directly connects each timer input channel
	// to its corresponding external input.
	// For example, IC1 (channel 1) is directly linked to TI1 (input1), IC2 to TI2, and so on.
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;

	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;

	// For de-bouncing (0 means we don't need to wait for de-bouncing).
	sConfigIC.ICFilter = 0;

	// This API will set the TIMx_CCMR1 register.
	// CCMR stands for Capture/Compare Mode Register.
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
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

