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

extern char at_cmd[];
extern uint8_t response_at_cmd[];
extern uint8_t res_is_ok;
extern uint32_t prev_tick;
extern const uint32_t timeout;

void sim_huart_init(UART_HandleTypeDef *p_huart_sim, UART_HandleTypeDef *p_huart_log);
void sim_mqtt_gps_init(void);
void sim_transmit(const char *cmd);

#endif // SIM7600_H
