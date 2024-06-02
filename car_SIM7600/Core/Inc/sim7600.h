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



void SIM_MQTT_GPS_Init(void);

#endif // SIM7600_H
