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

void SystemClockConfig(uint8_t clock_freq);
void Error_Handler(void);

void TIMER2_Init(void);
void MC01_Configuration(void);

// UART helpers.
void UART2_Init(void);
void logUART(const char *format, ...);

UART_HandleTypeDef huart2;

// Timer 2 and captures variable.
TIM_HandleTypeDef htimer2;
uint32_t input_captures[2] = { 0 };
uint8_t count = 1;

volatile uint32_t pulse1_value = 25000; //to produce 500Hz
volatile uint32_t pulse2_value = 12500; //to produce 1000HZ
volatile uint32_t pulse3_value = 6250;  //to produce 2000Hz
volatile uint32_t pulse4_value = 3125;  //to produce 4000Hz

volatile uint32_t ccr_content;

int main(void) {

	HAL_Init();

	// Use default system config.
	SystemClockConfig(SYS_CLOCK_FREQ_50_MHZ);

	// Initialize peripherals.
	UART2_Init();

	while (1) {

	}

	return 0;
}

void TIMER2_Init(void) {

	// Step 1. Initialize the timer with `Input Capture` mode.
	// Initialize timer.
	htimer2.Instance = TIM2;

	// Let it count up to the maximum value.
	htimer2.Init.CounterMode = TIM_COUNTERMODE_UP;

	// Check in reference manual to check the maximum capacity of the counter of this general purpose timer.
	// Timer 2 is 32 bit counter.
	htimer2.Init.Period = 0XFFFFFFFF;

	// Divide the timer clock.
	// Timer count clock will be divided by this value +1. So our timer2 will be 25Mhz.
	htimer2.Init.Prescaler = 1;

	// Initialize timer with INPUT CAPTURE instead of base.
	if (HAL_TIM_IC_Init(&htimer2) != HAL_OK) {
		Error_Handler();
	}

	// Step 2. Configure the input capture.
	TIM_OC_InitTypeDef timer2OC_Config;

	timer2OC_Config.OCMode = TIM_OCMODE_TOGGLE;

	// Select edge to read the signal.
	timer2OC_Config.OCPolarity = TIM_OCNPOLARITY_HIGH;

	timer2OC_Config.Pulse = pulse1_value;


	// Configure the 4 channels
	if (HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_handler();
	}

	timer2OC_Config.Pulse = pulse2_value;
	if (HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_handler();
	}

	timer2OC_Config.Pulse = pulse3_value;
	if (HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_handler();
	}

	timer2OC_Config.Pulse = pulse4_value;
	if (HAL_TIM_OC_ConfigChannel(&htimer2, &timer2OC_Config, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_handler();
	}
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

void SystemClockConfig(uint8_t clock_freq) {
	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;
	uint8_t flash_latency = 0;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_HSI;
	Osc_Init.HSEState = RCC_HSE_ON;
	Osc_Init.LSEState = RCC_LSE_ON;
	Osc_Init.HSIState = RCC_HSI_ON;
	Osc_Init.PLL.PLLState = RCC_PLL_ON;
	Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch (clock_freq) {
	case SYS_CLOCK_FREQ_50_MHZ:
		Osc_Init.PLL.PLLM = 4;
		Osc_Init.PLL.PLLN = 50;
		Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		Osc_Init.PLL.PLLQ = 2;

		Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
				| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
		flash_latency = FLASH_ACR_LATENCY_1WS;
		break;

	default:
		return;
	}

	if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK) {
		Error_Handler();
	}

	// Configure the systick timer interrupt frequency (for every 1 ms)
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	// Configure the Systick.
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	// SysTick_IRQn interrupt configuration
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

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

