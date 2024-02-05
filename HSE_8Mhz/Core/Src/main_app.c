/*
 * main_app.c
 *
 *  Created on: Jan 23, 2024
 *      Author: sherly hartono
 */

#include <string.h>
#include "main_app.h"
#include <stdio.h>

#define TRUE 1
#define FALSE 0

void SystemClockConfig(void);
void UART2_Init(void);
void Error_Handler(void);

// Handler of UART2
UART_HandleTypeDef huart2;

char *welcome_msg = "The application HSE_8Mhz is running.\r\n";

int main(void) {
	// 1. Oscillator (actual clock)
	RCC_OscInitTypeDef osc_init;

	// 2. Clock for the bus
	RCC_ClkInitTypeDef clk_init;
	char msg[100];

	// 3. Init UART and Hardware abstraction API.
	HAL_Init();
	UART2_Init();

	// 4. Initialize Oscillator.
	// Remove garbage in osc_init by memset to 0.
	memset(&osc_init, 0, sizeof(osc_init));
	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.HSEState = RCC_HSE_ON;

	// - Call the api with our struct.
	// It will wait until the oscillator is ready.
	if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
		Error_Handler();
	}

	// 5. Initialize the CPU, AHB and APB buses clocks
	clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
	RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;

	// - Set the prescaler or divider value for AHB and APB bus.
	clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
	clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}

	/*---------------------------- AFTER THIS LINE SYSCLK is SOURCED BY HSE------------------*/

	__HAL_RCC_HSI_DISABLE(); //Saves some current because we are now using HSE.

	/* LETS REDO THE SYSTICK CONFIGURATION */
	// Generate interrupt at every 1ms.
	// init systick, enable interrupt, we are overriding the default
	// previously set by HAL_Init() since we now have different HCLK.
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	// Set the MCU source clock for SYSTICK.
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	// Update UART since the APB clock has been changed.
	UART2_Init();

	// Print and send from UART.
	HAL_UART_Transmit(&huart2, (uint8_t*) welcome_msg, strlen(welcome_msg),
			HAL_MAX_DELAY);

	memset(msg, 0, sizeof(msg));
	sprintf(msg, "SYSCLK : %ldHz\r\n", HAL_RCC_GetSysClockFreq());
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	memset(msg, 0, sizeof(msg));
	sprintf(msg, "HCLK   : %ldHz\r\n", HAL_RCC_GetHCLKFreq());
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	memset(msg, 0, sizeof(msg));
	sprintf(msg, "PCLK1  : %ldHz\r\n", HAL_RCC_GetPCLK1Freq());
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	memset(msg, 0, sizeof(msg));
	sprintf(msg, "PCLK2  : %ldHz\r\n", HAL_RCC_GetPCLK2Freq());
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	return 0;
}

void SystemClockConfig(void) {
	/**
	 * Use internal oscillator if this function is empty.
	 */

}

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

	// 2. Initialize GPIO (we don't need to initialize handler). We can
	// iNitialize the uart direcyly in HAL_UART_MspInit.

}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
		/**
		 * maybe blink LED if there are errors.
		 */
	}
}

