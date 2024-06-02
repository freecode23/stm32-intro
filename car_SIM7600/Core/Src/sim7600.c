
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

char at_cmd[256];
uint8_t response_at_cmd[2000] = { };
uint8_t res_is_ok = 0;
uint32_t prev_tick;
const uint32_t timeout = 10000;


static UART_HandleTypeDef *huart_sim;
static UART_HandleTypeDef *huart_log;


void sim_huart_init(UART_HandleTypeDef *p_huart_sim, UART_HandleTypeDef *p_huart_log) {
    huart_sim = p_huart_sim;
    huart_log = p_huart_log;
}

/**
 * Send an AT command to SIM module, and log the command and response to UART.
 */
void sim_transmit(const char *cmd) {

	memset(response_at_cmd, 0, sizeof(response_at_cmd));

	// Send the AT command to SIM7600
	HAL_UART_Transmit(huart_sim, (uint8_t*) cmd, strlen(cmd), 2000);
	HAL_UART_Receive(huart_sim, (uint8_t*) response_at_cmd, sizeof(response_at_cmd),
			2000);

	// Log the response received by the SIM module.
	char status_msg[3000];
	sprintf(status_msg, "-->Command and res:\n%s\r\n", response_at_cmd);
	HAL_UART_Transmit(huart_log, (uint8_t*) status_msg, strlen(status_msg),
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
