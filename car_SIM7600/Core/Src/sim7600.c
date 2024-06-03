/*
 * sim7600.c
 *
 *  Created on: Jun 2, 2024
 *      Author: sherlyhartono
 */
#include "sim7600.h"

/**
 * Initialize the 5G provider, MQTT broker, and the topics we want to
 * publish or subscribe to.
 */
const char apn[] = "vzwinternet";
const char host[] = "tcp://apv3187879vov-ats.iot.us-east-2.amazonaws.com";
const int port = 8883;
char *will_message = "SIMCom Connected!";
const char topic_will[] = "topic/will";
const char topic_cmd[] = "topic/cmd";
const char topic_sensor[] = "topic/sensor";

// Variables needed for initializing the SIM module and connect to MQTT.
const uint32_t timeout = 10000;

// Variables needed to receive command.
uint8_t received_byte;
uint8_t cmd_buffer[200] = { }; // cmd_buffer stores all the command when it is received.
uint8_t cmd_buffer_index = 0;
char cmd_msg[100];
uint8_t cmd_msg_len;
volatile uint8_t cmd_received = 0;
volatile uint8_t receiving_cmd = 0;

// Variables needed to receive gpgga.
uint8_t gpgga_buffer[300] = { };
uint8_t gpgga_buffer_index = 0;
char gpgga_msg[300];
uint8_t gpgga_msg_len;
volatile uint8_t gpgga_received = 0;
volatile uint8_t receiving_gpgga = 0;

static UART_HandleTypeDef *huart_sim;
static UART_HandleTypeDef *huart_log;
static TIM_HandleTypeDef *pwm_timer;
uint32_t tim_channel;

// Private methods
static void send_AT_cmd(const char *cmd);
static void publish_mqtt_msg(char *msg, uint8_t msg_length);
static void extract_cmd(void);
static void extract_gpgga(void);

/**
 * Enable the stm32 to send message to sim module and to log the command or sensor messages to UART.
 */
void sim_huart_init(UART_HandleTypeDef *p_huart_sim,
		UART_HandleTypeDef *p_huart_log, TIM_HandleTypeDef *p_pwm_timer,
		uint32_t p_tim_channel) {
	huart_sim = p_huart_sim;
	huart_log = p_huart_log;
	pwm_timer = p_pwm_timer;
	tim_channel = p_tim_channel;
}

/**
 * Initialize MQTT publisher and subscriber
 * set to receive GPS info from SIM_7600.
 */
void sim_mqtt_gps_init(void) {

	uint8_t response_at_ok = 0;
	uint32_t prev_tick = HAL_GetTick();
	uint8_t response_at_cmd[2000] = { };

	// 0. Reset connection.
	while (!response_at_ok && prev_tick + timeout > HAL_GetTick()) {
		send_AT_cmd("AT+CRESET\r\n");
		if (strstr((char*) response_at_cmd, "SMS DONE")) {
			response_at_ok = 1;
		}
		HAL_Delay(1000);
	}

	// 1. Check for OK response for AT
	response_at_ok = 0;
	prev_tick = HAL_GetTick();
	while (!response_at_ok && prev_tick + timeout > HAL_GetTick()) {
		send_AT_cmd("AT\r\n");
		if (strstr((char*) response_at_cmd, "OK")) {
			response_at_ok = 1;
		}
		HAL_Delay(1000);
	}

	// 2. Check the certificate list
	send_AT_cmd("AT+CCERTLIST\r\n");

	// 3. Configure SSL with certificates.
	send_AT_cmd("AT+CSSLCFG=\"sslversion\",0,4\r\n");
	send_AT_cmd("AT+CSSLCFG=\"authmode\",0,2\r\n");
	send_AT_cmd("AT+CSSLCFG=\"cacert\",0,\"aws1_ca.pem\"\r\n");
	send_AT_cmd("AT+CSSLCFG=\"clientcert\",0,\"aws1_cert.pem\"\r\n");
	send_AT_cmd("AT+CSSLCFG=\"clientkey\",0,\"aws1_private.pem\"\r\n");

	// 4. Generate client and will topic
	send_AT_cmd("AT+CMQTTSTART\r\n");
	send_AT_cmd("AT+CMQTTACCQ=0,\"SIMCom_client01\",1\r\n");
	send_AT_cmd("AT+CMQTTSSLCFG=0,0\r\n");

	// 5. Set the Will Topic
	char at_cmd[256];
	sprintf(at_cmd, "AT+CMQTTWILLTOPIC=0,%d\r\n", strlen(topic_will));
	send_AT_cmd(at_cmd);
	send_AT_cmd(topic_will);

	// 6. Set the Will Message
	sprintf(at_cmd, "AT+CMQTTWILLMSG=0,%d,1\r\n", strlen(will_message));
	send_AT_cmd(at_cmd);
	send_AT_cmd(will_message);

	// 7. Connect to aws.
	sprintf(at_cmd, "AT+CMQTTCONNECT=0,\"%s:%d\",60,1\r\n", host, port);
	send_AT_cmd(at_cmd);

	// 8. Subscribe to "topic/cmd"
	sprintf(at_cmd, "AT+CMQTTSUBTOPIC=0,%d,1\r\n", strlen(topic_cmd));
	send_AT_cmd(at_cmd);
	sprintf(at_cmd, "%s\r\n", topic_cmd);
	send_AT_cmd(at_cmd);
	send_AT_cmd("AT+CMQTTSUB=0\r\n");

	// 9. GPS
	send_AT_cmd("AT+CGPS=0\r\n");

	//Configure GNSS support mode
	send_AT_cmd("AT+CGNSSMODE=15,1\r\n");

	// Configure NMEA sentence type
	send_AT_cmd("AT+CGPSNMEA=1\r\n");

	// Set NMEA output rate to 10Hz
	send_AT_cmd("AT+CGPSNMEARATE=1\r\n");
	send_AT_cmd("AT+CGPS=1\r\n");

	// NMEA Output to AT port
	send_AT_cmd("AT+CGPSINFOCFG=2,1\r\n");

	// 10. Ready to receive command from AWS byte by byte.
	HAL_UART_Receive_IT(huart_sim, &received_byte, 1);
}

/**
 * sim_handle_byte will decide what to do to a byte received from the sim module.
 * the byte can either belong to a gpgga string or a command.
 */
void sim_handle_byte() {

	// Check if we are just starting to receive gpgga or a command.
	if (received_byte == '$') {
		receiving_gpgga = 1;

	} else if (received_byte == '{') {
		receiving_cmd = 1;
	}

	if (receiving_cmd == 1) {
		if (received_byte == '}') {
			// Command completed.
			cmd_received = 1;
			receiving_cmd = 0;
			extract_cmd();
		} else {
			// Command not yet completed.
			cmd_buffer[cmd_buffer_index++] = received_byte;
		}

	} else if (receiving_gpgga == 1) {
		if (received_byte == '\n') {
			// GPGGA string completed.
			gpgga_received = 1;
			receiving_gpgga = 0;
			extract_gpgga();
		} else {
			// string not yet completed.
			gpgga_buffer[gpgga_buffer_index++] = received_byte;
		}
	}
	HAL_UART_Receive_IT(huart_sim, &received_byte, 1);
}

/**
 * Process the data after all bytes have been received.
 */
void sim_process_received_data(void) {

	// 1. Run motor command.
	if (cmd_received) {
		// Reset flag.
		cmd_received = 0;
		HAL_UART_Transmit(huart_log, (uint8_t*) cmd_msg, cmd_msg_len,
		HAL_MAX_DELAY);

		if (strstr((char*) cmd_msg, "forward")) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
			__HAL_TIM_SET_COMPARE(pwm_timer, tim_channel, 40000);

		} else if (strstr((char*) cmd_msg, "backward")) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			__HAL_TIM_SET_COMPARE(pwm_timer, tim_channel, 10000);

		} else if (strstr((char*) cmd_msg, "stop")) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
			__HAL_TIM_SET_COMPARE(pwm_timer, tim_channel, 0);

		}

		// Reset buffer.
		cmd_buffer_index = 0;
		cmd_msg_len = 0;
	}

	// 2. Publish GPGGA message to MQTT.
	if (gpgga_received) {
		// Reset flag.
		gpgga_received = 0;

		publish_mqtt_msg(gpgga_msg, gpgga_msg_len);

		// Reset buffer.
		gpgga_buffer_index = 0;
		gpgga_msg_len = 0;
	}
}


/**
 * Send an AT command to SIM module, and log the command and response to UART.
 */
static void send_AT_cmd(const char *cmd) {

	// Send the AT command to SIM7600.
	HAL_UART_Transmit(huart_sim, (uint8_t*) cmd, strlen(cmd), 2000);

	uint8_t response_at_cmd[2000] = { };
	HAL_UART_Receive(huart_sim, (uint8_t*) response_at_cmd,
			sizeof(response_at_cmd), 2000);

	// Log the response received by the SIM module.
	char status_msg[3000];
	sprintf(status_msg, "-->Command and res:\n%s\r\n", response_at_cmd);
	HAL_UART_Transmit(huart_log, (uint8_t*) status_msg, strlen(status_msg),
	HAL_MAX_DELAY);
}


/**
 * Publish message to mqtt topic.
 * The topic is currently default to topic/sensor.
 */
static void publish_mqtt_msg(char *msg, uint8_t msg_length) {

	// Create JSON message with GPGGA data
	uint8_t json_msg_len = msg_length + strlen("{\n\"message\":\"\"\n}");
	char json_msg[json_msg_len];
	sprintf(json_msg, "{\n\"message\":\"%s\"\n}", msg);

	// Tell SIM that we will be sending a a message under this topic.
	char at_cmd[256];
	sprintf(at_cmd, "AT+CMQTTTOPIC=0,%d\r\n", strlen(topic_sensor));
	send_AT_cmd(at_cmd);
	sprintf(at_cmd, "%s\r\n", topic_sensor);
	send_AT_cmd(at_cmd);

	// Define the payload.
	sprintf(at_cmd, "AT+CMQTTPAYLOAD=0,%d\r\n", strlen(msg));
	send_AT_cmd(at_cmd);
	send_AT_cmd(msg);

	// Publish the message.
	sprintf(at_cmd, "AT+CMQTTPUB=0,1,%d\r\n", strlen(msg));
	send_AT_cmd(at_cmd);
}



/**
 * Extract only the relevant string from all the command bytes received.
 */
static void extract_cmd(void) {

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

	// Copy the message from buffer to final array.
	cmd_msg_len = end_i - start_i;
	if (cmd_msg_len > sizeof(cmd_msg) - 1) {
		cmd_msg_len = sizeof(cmd_msg) - 1; // Prevent buffer overflow
	}

	strncpy(cmd_msg, (char*) cmd_buffer + start_i, cmd_msg_len);
	cmd_msg[cmd_msg_len] = '\n';
	cmd_msg[cmd_msg_len + 1] = '\0';
	cmd_msg_len++;

}

/**
 * Extract only the relevant string from all the gpgga bytes received.
 */
static void extract_gpgga(void) {

	// Copy the message from buffer to final array.
	gpgga_msg_len = gpgga_buffer_index;
	strncpy(gpgga_msg, (char*) gpgga_buffer, gpgga_msg_len);
	gpgga_msg[gpgga_msg_len] = '\0';

}

