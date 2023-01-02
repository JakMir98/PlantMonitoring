#include <stdio.h>
#include <stdlib.h>
#include <string.h> //Requires by memset

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include <driver/adc.h>
#include "driver/i2c.h"

#include "nvs_flash.h"
#include <netdb.h>

#include "ads1115.h"
#include "bmp180.h"
#include "constants.h"
#include "httpServer.h"

const int POT_PIN = 5;
#define REFERENCE_PRESSURE 101325l

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

static const char *TAG = "JMBW_ESP32"; // TAG for debug
Measurements measurements = 
{
    .bmpPressure = 1000.0,
    .bmpTemperature = 23.4,
    .bmpAltitude = 56.5,
    .soilMoisture1 = 15.1,
    .soilMoisture2 = 16.6
};
static SemaphoreHandle_t mutex;

void i2c_master_init()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000};
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

ads1115_t ads1115_cfg = {
  .reg_cfg =  ADS1115_CFG_LS_COMP_MODE_TRAD | // Comparator is traditional
              ADS1115_CFG_LS_COMP_LAT_NON |   // Comparator is non-latching
              ADS1115_CFG_LS_COMP_POL_LOW |   // Alert is active low
              ADS1115_CFG_LS_COMP_QUE_DIS |   // Compator is disabled
              ADS1115_CFG_LS_DR_1600SPS |     // No. of samples to take
              ADS1115_CFG_MS_MODE_SS,         // Mode is set to single-shot
  .dev_addr = 0x48,
};

void bmp180_measurements_task(void *pvParameter)
{
    while(1) {
        esp_err_t err;
        uint32_t pressure;
        uint8_t flags = 0;
        float altitude;
        float temperature;

        err = bmp180_read_pressure(&pressure);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "Reading of pressure from BMP180 failed, err = %d", err);
        }
        else
        {
            flags |= 1 << 0;
        }

        err = bmp180_read_altitude(REFERENCE_PRESSURE, &altitude);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "Reading of altitude from BMP180 failed, err = %d", err);
        }
        else
        {
            flags |= 1 << 1;
        }

        err = bmp180_read_temperature(&temperature);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG, "Reading of temperature from BMP180 failed, err = %d", err);
        }
        else
        {
            flags |= 1 << 2;
        }

        if (xSemaphoreTake(mutex, 1000 / portTICK_PERIOD_MS) == pdTRUE) // oczekiwanie 1s
        {
            if(flags & (1<<0))
            {
                measurements.bmpPressure = pressure;
            }

            if(flags & (1<<1))
            {
                measurements.bmpAltitude = altitude;
            }

            if(flags & (1<<2))
            {
                measurements.bmpTemperature = temperature;
            }
            
            xSemaphoreGive(mutex);
        }

        ESP_LOGI(TAG, "Pressure %d Pa, Altitude %.1f m, Temperature : %.1f oC", pressure, altitude, temperature);
        vTaskDelay(2500 / portTICK_PERIOD_MS);
    }
    vTaskDelete( NULL );
}

void analog_measurements_task(void * params)
{
    uint16_t firstChannelResult = 0;
    uint16_t secondChannelResult = 0;

    while(1)
    {
        // Request single ended on pin AIN0  
        ADS1115_request_single_ended_AIN0();      // all functions except for get_conversion_X return 'esp_err_t' for logging
        // Check conversion state - returns true if conversion is complete 
        while(!ADS1115_get_conversion_state()) 
        {
            vTaskDelay(pdMS_TO_TICKS(5));          // wait 5ms before check again
        }
        firstChannelResult = ADS1115_get_conversion(); 
        // Return latest conversion value  
        ESP_LOGI(TAG, "Conversion Value POT: %d", firstChannelResult);

        ADS1115_request_single_ended_AIN1(); 
        while(!ADS1115_get_conversion_state()) 
        {
            vTaskDelay(pdMS_TO_TICKS(5));          
        }
        secondChannelResult = ADS1115_get_conversion();   
        ESP_LOGI(TAG, "Soil 2: %d", secondChannelResult);

        if (xSemaphoreTake(mutex, 1000 / portTICK_PERIOD_MS) == pdTRUE) // oczekiwanie 1s
        {
            measurements.soilMoisture1 = firstChannelResult;
            measurements.soilMoisture2 = secondChannelResult;
            xSemaphoreGive(mutex);
        }
        vTaskDelay(2500 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    connect_wifi();

    setup_server();
    i2c_master_init();
    ADS1115_initiate(&ads1115_cfg);

    mutex = xSemaphoreCreateMutex();

    xTaskCreate(&analog_measurements_task, "analog_measurements_task", 4096, NULL, 5, NULL);

    ret = bmp180_init(SDA_PIN, SCL_PIN);
    if(ret == ESP_OK)
    {
        xTaskCreate(&bmp180_measurements_task, "bmp180_measurements_task", 4096, NULL, 5, NULL);
    } 
    else 
    {
        ESP_LOGE(TAG, "BMP180 init failed with error = %d", ret);
        measurements.bmpPressure = -999.0;
        measurements.bmpTemperature = -999.0;
    }
}
