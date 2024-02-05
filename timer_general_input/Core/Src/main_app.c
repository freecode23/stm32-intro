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
volatile uint8_t is_capture_done = FALSE;

int main(void) {

	HAL_Init();

	// Use default system config.
	SystemClockConfig(SYS_CLOCK_FREQ_50_MHZ);

	// Initialize peripherals.
	UART2_Init();

	// Init MCO1 so that timer 2 can capture input and
	// compute the timer period the clock signal sourced to MC01.
	MC01_Configuration();

	// Initialize input capture and start the timer.
	TIMER2_Init();

	logUART("init timer 2 success\r\n");
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
	htimer2.Init.Period = 4294967295;

	// Divide the timer clock.
	// Timer count clock will be divided by this value +1. So our timer2 will be 25Mhz.
	htimer2.Init.Prescaler = 1;


	htimer2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htimer2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	// Initialize timer with INPUT CAPTURE instead of base.
	if (HAL_TIM_IC_Init(&htimer2) != HAL_OK) {
		Error_Handler();
	}

	// Step 2. Configure the input capture.
	TIM_IC_InitTypeDef timer2IC_Config;


	// Select edge to read the signal.
	timer2IC_Config.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;

	// Select input Direct Capture:
	// This configuration directly connects each timer input channel
	// to its corresponding external input.
	// For example, IC1 (channel 1) is directly linked
	// to TI1 (input1), IC2 to TI2, and so on.
	timer2IC_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;

	timer2IC_Config.ICPrescaler = TIM_ICPSC_DIV1;

	// For de-bouncing (0 means we don't need to wait for de-bouncing).
	timer2IC_Config.ICFilter = 0;

	// We want the capture/compare of channel 1.
	// This function will set the TIMx_CCMR1 register.
	// CCMR stands for Capture/Compare Mode Register.
	if (HAL_TIM_IC_ConfigChannel(&htimer2, &timer2IC_Config, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * Configure MC01 to take in any desired clock source.
 */
void MC01_Configuration(void)
{
	// This will initialize the GPIO for PA8.
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
 * Callback when interrupt is triggered in input capture of timer2.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	logUART("HAL_TIM_IC_CaptureCallback\r\n");

	char usr_msg[100]; // Buffer for the formatted message
	sprintf(usr_msg,"HAL_TIM_IC_CaptureCallback\r\n" );
	HAL_UART_Transmit(&huart2,(uint8_t*)usr_msg, strlen(usr_msg),HAL_MAX_DELAY);
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

void SystemClockConfig(uint8_t clock_freq) {


}

void Error_Handler(void) {
	while (1) {
		logUART("ERROR!!");
	}

}

