#ifndef UTILITIES_H
#define UTILITIES_H

#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint16_t map(int x, int in_min, int in_max, int out_min, int out_max);
int map_soil_moisture(int adcValue);
int convert_soil_moisture_value(int soilMoisture);
unsigned int get_used_stack_size(int tStackSize, TaskHandle_t handle);

#endif