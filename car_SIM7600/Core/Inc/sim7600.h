// sim7600.h
#ifndef SIM7600_H
#define SIM7600_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>


// Define constants and variables
//#define CMD_BUFFER_SIZE 200
//#define CMD_MESSAGE_SIZE 100
//#define GPGGA_BUFFER_SIZE 300
//#define GPGGA_MESSAGE_SIZE 300
//
//
//typedef struct {
//    uint8_t *buffer;
//    uint8_t buffer_index;
//    char *msg;
//    uint8_t msg_len;
//    volatile uint8_t received;
//    volatile uint8_t receiving;
//} SIM7600_Message;
//
//typedef struct {
//    const char *apn;
//    const char *host;
//    int port;
//    char *will_message;
//    const char *topic_will;
//    const char *topic_cmd;
//    const char *topic_sensor;
//
//    const uint32_t timeout;
//    uint8_t received_byte;
//
//    SIM7600_Message cmd;
//    SIM7600_Message gpgga;
//
//    UART_HandleTypeDef *huart_sim;
//    UART_HandleTypeDef *huart_log;
//    TIM_HandleTypeDef *pwm_timer;
//    uint32_t tim_channel;
//
//} SIM7600_Context;
//
//extern SIM7600_Context sim_ctx;

// Public methods
void sim_huart_init(UART_HandleTypeDef *p_huart_sim,
		UART_HandleTypeDef *p_huart_log, TIM_HandleTypeDef *pwm_timer,
		uint32_t tim_channel);
void sim_mqtt_gps_init(void);
void sim_handle_byte();
void sim_process_received_data();

#endif // SIM7600_H
