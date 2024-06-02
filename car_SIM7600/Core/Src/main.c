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
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char at_cmd[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void sim_mqtt_gps_init(void);
void sim_transmit(const char *cmd);
static void publish_msg(char *msg, uint8_t msg_length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const char apn[] = "vzwinternet";

// Use tcp for non secure connection.
const char host[] = "tcp://apv3187879vov-ats.iot.us-east-2.amazonaws.com";
const int port = 8883;
char *will_message = "SIMCom Connected!";
const char topic_will[] = "topic/will";
const char topic_cmd[] = "topic/cmd";
const char topic_sensor[] = "topic/sensor";
volatile uint8_t cmd_received = 0;
volatile uint8_t receiving_cmd = 0;
volatile uint8_t gpgga_received = 0;
volatile uint8_t receiving_gpgga = 0;

const uint32_t timeout = 10000;

// response_at_cmd is a buffer for all the responses received when we initialize MQTT.
uint8_t response_at_cmd[2000] = { };
uint8_t res_is_ok = 0;
uint32_t prev_tick;

// received_byte is a single byte when command is published.
uint8_t received_byte;

// cmd_buffer stores all the command when it is received.
uint8_t cmd_buffer[200] = { };
uint8_t cmd_buffer_index = 0;
char cmd_msg[100];
uint8_t cmd_msg_len;

// cmd_buffer stores all the GPGGA string when received.
uint8_t gpgga_buffer[300] = { };
uint8_t gpgga_buffer_index = 0;
char gpgga_msg[300];
uint8_t gpgga_msg_len;

/**
 * Send an AT command to SIM module, and log the command and response to UART.
 */
void sim_transmit(const char *cmd) {

	memset(response_at_cmd, 0, sizeof(response_at_cmd));

	// Send the AT command to SIM7600
	HAL_UART_Transmit(&huart3, (uint8_t*) cmd, strlen(cmd), 2000);
	HAL_UART_Receive(&huart3, (uint8_t*) response_at_cmd, sizeof(response_at_cmd),
			2000);

	// Log the response received by the SIM module.
	char status_msg[3000];
	sprintf(status_msg, "-->Command and res:\n%s\r\n", response_at_cmd);
	HAL_UART_Transmit(&huart2, (uint8_t*) status_msg, strlen(status_msg),
	HAL_MAX_DELAY);
}

/**
 * Initialize MQTT publisher and subscriber
 * set to receive GPS info from SIM_7600.
 */
void sim_mqtt_gps_init(void) {
	// 0. Reset connection.
	res_is_ok = 0;
	prev_tick = HAL_GetTick();
	while (!res_is_ok && prev_tick + timeout > HAL_GetTick()) {
		sim_transmit("AT+CRESET\r\n");
		if (strstr((char*) response_at_cmd, "SMS DONE")) {
			res_is_ok = 1;
		}
		HAL_Delay(1000);
	}

	// 1. Check for OK response for AT
	res_is_ok = 0;
	prev_tick = HAL_GetTick();
	while (!res_is_ok && prev_tick + timeout > HAL_GetTick()) {
		sim_transmit("AT\r\n");
		if (strstr((char*) response_at_cmd, "OK")) {
			res_is_ok = 1;
		}
		HAL_Delay(1000);
	}

	// 2. Check the certificate list
	sim_transmit("AT+CCERTLIST\r\n");

	// 3. Configure SSL with certificates.
	sim_transmit("AT+CSSLCFG=\"sslversion\",0,4\r\n");
	sim_transmit("AT+CSSLCFG=\"authmode\",0,2\r\n");
	sim_transmit("AT+CSSLCFG=\"cacert\",0,\"aws1_ca.pem\"\r\n");
	sim_transmit("AT+CSSLCFG=\"clientcert\",0,\"aws1_cert.pem\"\r\n");
	sim_transmit("AT+CSSLCFG=\"clientkey\",0,\"aws1_private.pem\"\r\n");

	// 4. Generate client and will topic
	sim_transmit("AT+CMQTTSTART\r\n");
	sim_transmit("AT+CMQTTACCQ=0,\"SIMCom_client01\",1\r\n");
	sim_transmit("AT+CMQTTSSLCFG=0,0\r\n");

	// 5. Set the Will Topic
	sprintf(at_cmd, "AT+CMQTTWILLTOPIC=0,%d\r\n", strlen(topic_will));
	sim_transmit(at_cmd);
	sim_transmit(topic_will);

	// 6. Set the Will Message
	sprintf(at_cmd, "AT+CMQTTWILLMSG=0,%d,1\r\n", strlen(will_message));
	sim_transmit(at_cmd);
	sim_transmit(will_message);

	// 7. Connect to aws.
	sprintf(at_cmd, "AT+CMQTTCONNECT=0,\"%s:%d\",60,1\r\n", host, port);
	sim_transmit(at_cmd);

	// 8. Subscribe to "topic/cmd"
	sprintf(at_cmd, "AT+CMQTTSUBTOPIC=0,%d,1\r\n", strlen(topic_cmd));
	sim_transmit(at_cmd);
	sprintf(at_cmd, "%s\r\n", topic_cmd);
	sim_transmit(at_cmd);
	sim_transmit("AT+CMQTTSUB=0\r\n");

	// 9. GPS
	sim_transmit("AT+CGPS=0\r\n");
	//Configure GNSS support mode
	sim_transmit("AT+CGNSSMODE=15,1\r\n");
	// Configure NMEA sentence type
	sim_transmit("AT+CGPSNMEA=1\r\n");
	// Set NMEA output rate to 10Hz
	sim_transmit("AT+CGPSNMEARATE=1\r\n");
	sim_transmit("AT+CGPS=1\r\n");
	// NMEA Output to AT port
	sim_transmit("AT+CGPSINFOCFG=2,1\r\n");
}

static void publish_msg(char *msg, uint8_t msg_length) {

	// Create JSON message with GPGGA data
	uint8_t json_msg_len = msg_length
			+ strlen("{\n\"message\":\"\"\n}");
	char json_msg[json_msg_len];
	sprintf(json_msg, "{\n\"message\":\"%s\"\n}", msg);

	// Tell SIM that we will be sending a a message under this topic.
	sprintf(at_cmd, "AT+CMQTTTOPIC=0,%d\r\n", strlen(topic_sensor));
	sim_transmit(at_cmd);
	sprintf(at_cmd, "%s\r\n", topic_sensor);
	sim_transmit(at_cmd);

	// Define the payload.
	char at_cmd[256];
	sprintf(at_cmd, "AT+CMQTTPAYLOAD=0,%d\r\n", strlen(msg));
	sim_transmit(at_cmd);
	sim_transmit(msg);

	// Publish the message.
	sprintf(at_cmd, "AT+CMQTTPUB=0,1,%d\r\n", strlen(msg));
	sim_transmit(at_cmd);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	// This program will first initialize connection with AWS IoT MQTT broken.
	// If a message is published on topic/cmd, the message received from SIM7600 module will be received on uart3,
	// it will then log to uart3
	//
	// If a user button is pressed on STM32, it will publish a message on topic/sensor
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
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart2, (uint8_t*) "App started.\r\n",
			strlen("App started.\r\n"),
			HAL_MAX_DELAY);

	// 1. MQTT and GPS setup.
	sim_mqtt_gps_init();
	// Ready to receive command from AWS byte by byte.
	HAL_UART_Receive_IT(&huart3, &received_byte, 1);

	// 2. Timer4 PWM set up.
	if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
		HAL_UART_Transmit(&huart2,
				(uint8_t*) "Error initializing base timer\r\n",
				strlen("Error initializing base timer\r\n"),
				HAL_MAX_DELAY);
		Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4)) {
		HAL_UART_Transmit(&huart2,
				(uint8_t*) "Error initializing pwm timer\r\n",
				strlen("Error initializing pwm timer\r\n"),
				HAL_MAX_DELAY);
		Error_Handler();
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		// 1. Log command from SIM module
		if (cmd_received) {
			cmd_received = 0;

			// Log the response received by the SIM module.
			HAL_UART_Transmit(&huart2, (uint8_t*) cmd_msg, cmd_msg_len,
			HAL_MAX_DELAY);

			if (strstr((char*) cmd_msg, "forward")) {
				// Front Left
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 40000);

			} else if (strstr((char*) cmd_msg, "backward")) {
				// Front Left
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 10000);


			} else if (strstr((char*) cmd_msg, "stop")) {
				// Front Left
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

			}

			// Ready to receive next command.
			cmd_buffer_index = 0;
			cmd_msg_len = 0;
		}

		// 2. Send GPGGA string to MQTT
		if (gpgga_received) {
			gpgga_received = 0;


			// Publish and ready to receive next command.
			publish_msg(gpgga_msg, gpgga_msg_len);
			gpgga_buffer_index = 0;
			gpgga_msg_len = 0;
		}

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
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */
	// ARR is 49999
	// Timer 4 clock is 16Mhz
	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 4;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 49999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

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
	HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | Audio_RST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

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

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | Audio_RST_Pin;
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

	/*Configure GPIO pins : PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

/**
 * Call back function everytime we receive a single byte in UART.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	// Received a byte from SIM7600 module.
	if (huart == &huart3) {

		// Check which string we are receiving
		if (received_byte == '$') {
			receiving_gpgga = 1;

		} else if (received_byte == '{') {
			receiving_cmd = 1;
		}

		// Case 2: We are receiving command.
		if (receiving_cmd == 1) {

			// 2A: Command completed.
			if (received_byte == '}') {
				cmd_received = 1;
				receiving_cmd = 0;
				cmd_buffer[cmd_buffer_index++] = '\0';

				// Find the command string e.g. "forward", "backward", etc.
				// Get the start and and index of the message
				// end index is the index of the last quote
				uint8_t end_i = cmd_buffer_index - 2;
				uint8_t start_i = end_i - 1;
				for (; start_i > 0; start_i--) {
					// if character is not a quote, skip.
					if (cmd_buffer[start_i] != '"') {
						continue;
					}
					// start_i now points to the first character.
					start_i += 1;
					break;
				}

				// Copy the message from buffer and store to cmd_msg.
				cmd_msg_len = end_i - start_i;
				if (cmd_msg_len > sizeof(cmd_msg) - 1) {
					cmd_msg_len = sizeof(cmd_msg) - 1; // Prevent buffer overflow
				}

				strncpy(cmd_msg, (char*) cmd_buffer + start_i,
						cmd_msg_len);
				cmd_msg[cmd_msg_len] = '\n';
				cmd_msg[cmd_msg_len + 1] = '\0';
				cmd_msg_len++;

			} else {
				// 2B: Command not yet completed.
				// append the byte to buffer
				cmd_buffer[cmd_buffer_index++] = received_byte;
			}

			// Case 3: We are receiving GPGGA.
		} else if (receiving_gpgga == 1) {

			if (received_byte == '\n') {
				// 3A: GPGGA string completed.
				// Clear buffer and move it to message array.
				gpgga_received = 1;
				receiving_gpgga = 0;

				// Print the last char before we meet `\n`
//				char lastCharMsg[30];
//				snprintf(lastCharMsg, sizeof(lastCharMsg), "lastChar=%cEND\n", gpgga_msg[gpgga_buffer_index - 1]);
//				HAL_UART_Transmit(&huart2, (uint8_t*) lastCharMsg, strlen(lastCharMsg), HAL_MAX_DELAY);

				// Question: why do I need to do -1 to make it not have \n at the end of GPGGA string.
				// This should say that the length is bufferIndex=11 not bufferIndex-1=10
				// Buffer content: ['$', 'G', 'P', 'G', 'G', 'A', ',', ',', '1', '2', '3', '\n']
				// Buffer indexes:   0    1    2    3    4    5    6    7    8    9   10    11
				// 																			 *
				//  																			we are now here.
				// Copy buffer to the final message.
				gpgga_msg_len = gpgga_buffer_index;
				strncpy(gpgga_msg, (char*) gpgga_buffer, gpgga_msg_len);
				gpgga_msg[gpgga_msg_len] = '\0';

			} else {
				// 3B: string not yet completed.
				gpgga_buffer[gpgga_buffer_index++] = received_byte;
			}
		}

		HAL_UART_Receive_IT(&huart3, &received_byte, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) // Check if the interrupt comes from the button pin
	{
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
