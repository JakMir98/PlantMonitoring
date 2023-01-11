#include "utilities.h"

static const int MAX_16BIT_VALUE = 65536;
static const int AIR_VALUE = -90;  
static const int WATER_VALUE = 5224;

static const char *TAG = "UTILITIES_JMBW_ESP32"; // TAG for debug

uint16_t map(int x, int in_min, int in_max, int out_min, int out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int map_soil_moisture(int adcValue)
{
    int soilPercent = map(adcValue, AIR_VALUE, WATER_VALUE, 0, 100);

    if(soilPercent > 100)
    {
        soilPercent = 100;
    }
    else if (soilPercent < 0)
    {
        soilPercent = 0;
    }

    return soilPercent;
}

int convert_soil_moisture_value(int soilMoisture)
{
    if(soilMoisture > 65000) soilMoisture = soilMoisture - MAX_16BIT_VALUE;
    ESP_LOGI(TAG, "Soil: %d", soilMoisture);
    return map_soil_moisture(soilMoisture);
}

unsigned int get_used_stack_size(int tStackSize, TaskHandle_t handle) 
{
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    unsigned int usedStack = tStackSize - uxHighWaterMark/4;
    return usedStack;
}
