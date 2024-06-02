// sim7600.h
#ifndef SIM7600_H
#define SIM7600_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// Define constants and variables
extern const char apn[];
extern const char host[];
extern const int port;
extern char *will_message;
extern const char topic_will[];
extern const char topic_cmd[];
extern const char topic_sensor[];

// received_byte is a single byte we received from SIM module.
// This can be either a command or a GPGGA strings.
extern uint8_t received_byte;

// Variables needed for initializing the SIM module and connect to MQTT.
extern char at_cmd[];
extern uint8_t response_at_cmd[];
extern uint8_t res_is_ok;
extern uint32_t prev_tick;
extern const uint32_t timeout;

// Variables needed to receive command.
extern uint8_t cmd_buffer[]; // cmd_buffer stores all the command when it is received.
extern uint8_t cmd_buffer_index;
extern char cmd_msg[];
extern uint8_t cmd_msg_len;
extern volatile uint8_t cmd_received;
extern volatile uint8_t receiving_cmd;



void sim_huart_init(UART_HandleTypeDef *p_huart_sim, UART_HandleTypeDef *p_huart_log);
void sim_mqtt_gps_init(void);
void sim_transmit(const char *cmd);
void extract_cmd(void);

#endif // SIM7600_H
