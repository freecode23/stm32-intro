/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t send_sms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char apn[] = "vzwinternet";

// Use tcp for non secure conenction.
const char host[] = "tcp://86e793eba99948c8822845a7124e3aec.s1.eu.hivemq.cloud";
const int port = 1883;
const char username[] = "";
const char password[] = "";
const char topic_cmd[] = "topic/cmd";
const char topic_sensor[] = "topic/sensor";
const uint32_t timeOut = 10000;
uint8_t rxBuffer[160] = { };
// Buffer to store the response
uint8_t ATisOK = 0;
uint8_t CGREGisOK = 0;
uint32_t previousTick;
uint16_t readValue;
char charData[15];

/**
 * Send an AT command to SIM module, and log the command and response to UART.
 */
void SIMTransmit(char *cmd) {

	// Clear the response buffer.
	memset(rxBuffer, 0, sizeof(rxBuffer));

	// Send the AT command to SIM7600.
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart2, (uint8_t*) rxBuffer, sizeof(rxBuffer), 5000);

	// Log the response received by the SIM module.
	char status_msg[200];
	sprintf(status_msg, "-->Command and res:\n%s\r\n", rxBuffer);
	HAL_UART_Transmit(&huart3, (uint8_t*) status_msg, strlen(status_msg),
	HAL_MAX_DELAY);

}

uint8_t SIMTransmitWithPrompt(const char *cmd, const char *additionalData) {
	// Clear the receive buffer
	memset(rxBuffer, 0, sizeof(rxBuffer));

	// Send the AT command.
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);
	uint32_t startTime = HAL_GetTick();

	// Loop until prompt is received or timeout occurs
	char status_msg[200];
	while (strstr((char*) rxBuffer, ">") == NULL) {
		HAL_UART_Receive(&huart2, (uint8_t*) rxBuffer, sizeof(rxBuffer), 5000);

		// Check timeout
		if ((HAL_GetTick() - startTime) > 5000) {
			sprintf(status_msg, "-->Time out!\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t*) status_msg,
					strlen(status_msg), HAL_MAX_DELAY);
			break;
		}
	}

	// Log the response from sending AT command.
	sprintf(status_msg, "-->Initial Response:\n%s\r\n", rxBuffer);
	HAL_UART_Transmit(&huart3, (uint8_t*) status_msg, strlen(status_msg),
			HAL_MAX_DELAY);

	// Check if the response include the prompt ">"
	if (strstr((char*) rxBuffer, ">") != NULL) {
		HAL_UART_Transmit(&huart2, (uint8_t*) additionalData,
				strlen(additionalData), HAL_MAX_DELAY);

		// Clear the buffer again to receive the final response
		memset(rxBuffer, 0, sizeof(rxBuffer));
		HAL_UART_Receive(&huart2, (uint8_t*) rxBuffer, sizeof(rxBuffer), 5000);

		// Log the final response.
		sprintf(status_msg, "-->Final response:\n%s\r\n", rxBuffer);
		HAL_UART_Transmit(&huart3, (uint8_t*) status_msg, strlen(status_msg),
				HAL_MAX_DELAY);
		return 1;

	} else {

		sprintf(status_msg, "-->Prompt not found\r\n");
		return 0;
	}
}

void mqttPublish(void) {
	ATisOK = 0;
	CGREGisOK = 0;
	char ATcommand[256];
	char additionalData[256];

	// 1. Check for OK response for AT
	previousTick = HAL_GetTick();
	while (!ATisOK && previousTick + timeOut > HAL_GetTick()) {
		SIMTransmit("AT\r\n");
		if (strstr((char*) rxBuffer, "OK")) {
			ATisOK = 1;
		}

		HAL_Delay(1000);

	}

	// 2. Ensure GPRS network is available.
	if (ATisOK) {
		previousTick = HAL_GetTick();
		while (!CGREGisOK && previousTick + timeOut > HAL_GetTick()) {
			SIMTransmit("AT+CSQ\r\n");
			SIMTransmit("AT+CREG?\r\n"); // voice/data calls and SMSs
			SIMTransmit("AT+CGREG?\r\n"); // network allowing the access to internet.
			if (strstr((char*) rxBuffer, "+CGREG: 0,1")) {
				CGREGisOK = 1;
			}
		}
	}

	// If registered
	if (CGREGisOK) {
		// 1.Ensure GPRS network is available.
		sprintf(ATcommand, "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r\n", apn);
		SIMTransmit(ATcommand);
		SIMTransmit("AT+CGPADDR\r\n");

		// 2. Start MQTT service, activate PDP context
		SIMTransmit("AT+CMQTTSTART\r\n");

		// 3. Acquire a client.
		SIMTransmit("AT+CMQTTACCQ=0,\"client01\"\r\n");

		// 4. Connect to MQTT server
		sprintf(ATcommand, "AT+CMQTTCONNECT=0,\"%s:%d\",60,1\r\n", host, port);
		SIMTransmit(ATcommand);


		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

		// 5. Set the topic for publish message "topic/cmd".
		// Question1: How to send a multipart message?
		// When we send the first command to set the topic, we are suppposed to receive a ">" as reposnse.
		// Once we receive that response, we should follow up and send the name of the topic.
		// What is the best way to do this ?
		// I noticed that AT+CGREG gives me 0,3 reponse instead of 0,1. Could this be the reason?
		// I have tried 2 options:
		// Option 1: Use \r\n in both command and topic and don't bother to use while loop to check whether we have receivied the prompt signal ">".
		// I see this working in a youtuber video.
		sprintf(ATcommand,"AT+CMQTTTOPIC=0,%d",strlen(topic_cmd));
		SIMTransmit(ATcommand);
		sprintf(ATcommand,"%s\x1A",topic_cmd);
		SIMTransmit(ATcommand);

		// Option 2: Use while loop and wait for the ">" sign before sending the topic name.
		// This also doesn't work. It always hits timeout.
//		sprintf(ATcommand,"AT+CMQTTTOPIC=0,%d", strlen(topic_cmd));
//		sprintf(additionalData,"%s\x1A", topic_cmd);
//		SIMTransmitWithPrompt(ATcommand, additionalData);

		// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// Question 2. Why do I need to always restart the SIM module in order to not have the followiing error:
//		-->Command and res:
//		AT+CGPADDR
//
//		+CGPADDR: 1,100.111.74.74
//		+CGPADDR: 2,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
//		+CGPADDR: 3,0.0.0.0,0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0
//		+CGPADDR: 4,0...
//		-->Command and res:
//		0AT+CMQTTSTART
//
//		+CMQTTSTART: 23
//
//		ERROR

		// 6. Set the payload
		sprintf(ATcommand, "AT+CMQTTPAYLOAD=0,%d\r\n",
				strlen("Hello from sim7600"));
		SIMTransmit(ATcommand);
		SIMTransmit("Hello from sim7600");
//
//		// 7. Publish.
//		SIMTransmit("AT+CMQTTPUB=0,1,60\r\n");
//
//		// 8. Disconnect
//		SIMTransmit("AT+CMQTTDISC=0,120\r\n"); // Disconnect from Server
//		SIMTransmit("AT+CMQTTREL=0\r\n"); // Release the Client
//		SIMTransmit("AT+CMQTTSTOP\r\n"); // Stop MQTT Service

	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_UART_Transmit(&huart3, (uint8_t*) "App started.\r\n",
			strlen("App started.\r\n"),
			HAL_MAX_DELAY);

	mqttPublish();
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_I2C_SPI_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PDM_OUT_Pin */
	GPIO_InitStruct.Pin = PDM_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : I2S3_WS_Pin */
	GPIO_InitStruct.Pin = I2S3_WS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin | SPI1_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CLK_IN_Pin */
	GPIO_InitStruct.Pin = CLK_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
	GPIO_InitStruct.Pin = I2S3_MCK_Pin | I2S3_SCK_Pin | I2S3_SD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : VBUS_FS_Pin */
	GPIO_InitStruct.Pin = VBUS_FS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
	GPIO_InitStruct.Pin = OTG_FS_ID_Pin | OTG_FS_DM_Pin | OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
	GPIO_InitStruct.Pin = Audio_SCL_Pin | Audio_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == B1_Pin) // Check if the interrupt comes from the button pin
	{
		// If button is pressed, set the send SMS to SIM7600 to true.
		send_sms = 1;

	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
