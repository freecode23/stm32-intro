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

void SystemClockConfig(uint8_t clock_freq);
void UART2_Init(void);
void Error_Handler(void);

// Handler of UART2
UART_HandleTypeDef huart2;

char *user_data = "The application is running interrupt\r\n";

int main(void) {

	// Init UART and Hardware abstraction API.
	HAL_Init();
	UART2_Init();


	SystemClockConfig();


	return 0;
}

void SystemClockConfig(uint8_t clock_freq)
{
	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;

	uint32_t FLatency =0;

	// 1. Initialize oscillator.
	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	osc_init.HSIState = RCC_HSI_ON;
	osc_init.HSICalibrationValue = 16;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;

	switch(clock_freq)
	{
		// Config for SYS and HCLOCK 50Mhz:
		case SYS_CLOCK_FREQ_50_MHZ:
		{
			// Oscillator set up.
			osc_init.PLL.PLLM = 16;
			osc_init.PLL.PLLN = 100; // 50 <= PLLN <= 432
			osc_init.PLL.PLLP = 2;


			// Clocks set up.
		    clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
		    					RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

		    // To get HCLK.
		    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;

		    // To get PCLK.
		    clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		    clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

		    FLatency = FLASH_ACR_LATENCY_1WS;
			break;
		}
		case SYS_CLOCK_FREQ_84_MHZ:
		{
			// Oscillator setup.
			osc_init.PLL.PLLM = 16;
			osc_init.PLL.PLLN = 168;
			osc_init.PLL.PLLP = 2;
			osc_init.PLL.PLLQ  = 2;


			// Clock set up.
			clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
						RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
			clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
			clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

			// 2WS for 60 < HCLK <= 90
			FLatency = FLASH_ACR_LATENCY_2WS;

			break;
		}
		case SYS_CLOCK_FREQ_120_MHZ:
		{
			// Oscillator setup.
			osc_init.PLL.PLLM = 16;
			osc_init.PLL.PLLN = 240;
			osc_init.PLL.PLLP = 2;
			osc_init.PLL.PLLQ  = 2;


			// Clock set up.
			clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
						RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
			clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
			clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
			clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
			clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

			// Set FLASH latency according to HCLK
			// 3WS for 90 < HCLK <= 120
			FLatency = FLASH_ACR_LATENCY_3WS;
			break;

		}

		default:
			return;

	}

	// Initialize RCC with oscillator config.
	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK)
	{
		Error_handler();
	}

	// Initialize RCC with clock config.
	if(HAL_RCC_ClockConfig(&clk_init, FLatency) != HAL_OK)
	{
		Error_handler();
	}


	//Systick configuration
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);


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

}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
		/**
		 * maybe blink LED if there are errors.
		 */
	}
}


