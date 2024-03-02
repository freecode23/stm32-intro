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
void UART2_Init(void);
void Error_Handler(void);
uint8_t convert_to_capital(uint8_t);

// Handler of UART2
UART_HandleTypeDef huart2;

char *user_data = "The application is running\r\n";


int main(void) {
	// 1. Init hardware abstraction layer.
	// This will call msp init which will call the low level Processor
	// specific hardware initialization.
	HAL_Init();
	SystemClockConfig();

	// 2. Peripheral initialization.
	UART2_Init();

	// 3. Send data.
	uint16_t len_of_data = strlen(user_data);
	HAL_UART_Transmit(&huart2, (uint8_t*)user_data, len_of_data, HAL_MAX_DELAY);


	// 4. Create buffer for data received from UART buffer.
	uint8_t rcvd_data;
	uint8_t data_buffer[100];
	uint32_t byte_count=0;

	// 5. Convert to capital.
	while(1)
	{
		// Receive 1 byte at a time to rcvd_data buffer.
		// Make sure check local echo in terminal setting in CooolTerm
		// so we can send character over uart. make sure to receive CR and LF.
		HAL_UART_Receive(&huart2 ,&rcvd_data, 1, HAL_MAX_DELAY);
		if(rcvd_data == '\r') // If user press enter quite while loop.
		{
			break;
		}
		else
		{
			data_buffer[byte_count++]= convert_to_capital(rcvd_data);
		}
	}

	data_buffer[byte_count++]= '\r';

	// 6. Send back character.
	HAL_UART_Transmit(&huart2, data_buffer, byte_count, HAL_MAX_DELAY);

	while(1);

	return 0;
}

void SystemClockConfig(void)
{
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

uint8_t convert_to_capital(uint8_t data)
{
	if( data >= 'a' && data <= 'z')
	{
		data = data - ( 'a'- 'A');
	}

	return data;

}
