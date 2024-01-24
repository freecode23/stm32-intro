/*
 * main_app.c
 *
 *  Created on: Jan 23, 2024
 *      Author: sherly hartono
 */

#include <string.h>
#include "main_app.h"

#define TRUE 1
#define FALSE 0

void SystemClockConfig(void);
void UART2_Init(void);
void Error_Handler(void);
uint8_t convert_to_capital(uint8_t);
uint8_t recvd_data;
uint8_t data_buffer[100]; //
uint32_t byte_count=0;
uint8_t reception_complete = FALSE;

// Handler of UART2
UART_HandleTypeDef huart2;

char *user_data = "The application is running interrupt\r\n";

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

	// 4. Convert to capital Interrupt base.
    while(reception_complete != TRUE)
    {
    	// When received data is complete, UART will issue interrupt to processor
    	// and HAL_UART_IRQHandler(&huart2); in it.c will be called.
    	// And the interrupt will get processed under that HAL IRQ handler function.
    	HAL_UART_Receive_IT(&huart2, &recvd_data,1);
    }
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

/**
 * This function will be called when interrupt from UART is triggered.
 *
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if(recvd_data == '\r')
	 {
		 reception_complete = TRUE;
		 data_buffer[byte_count++]='\r'; // Make sure the last one is carriage return
		 HAL_UART_Transmit(huart, data_buffer, byte_count, HAL_MAX_DELAY);
	 }
	 else
	 {
		 data_buffer[byte_count++] = convert_to_capital(recvd_data);
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
