// sim7600.h
#ifndef SIM7600_H
#define SIM7600_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>


// Public methods
void sim_huart_init(UART_HandleTypeDef *p_huart_sim,
		UART_HandleTypeDef *p_huart_log, TIM_HandleTypeDef *pwm_timer,
		uint32_t tim_channel);
void sim_mqtt_gps_init(void);
void sim_handle_byte();
void sim_process_received_data();

#endif // SIM7600_H
