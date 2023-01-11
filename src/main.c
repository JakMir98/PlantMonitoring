#include <stdio.h>
#include <stdlib.h>
#include <string.h> 

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
#include "utilities.h"

#define DEBUG 0

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

const int REFERENCE_PRESSURE = 101325l;
static const char *TAG = "JMBW_ESP32"; // TAG for debug
Measurements measurements = 
{
    .bmpPressure = 102400.0,
    .bmpTemperature = 23.4,
    .bmpAltitude = 214.5,
    .soilMoisture1 = 22.1,
    .soilMoisture2 = 24.6
};
SemaphoreHandle_t mutex;
TaskHandle_t bmp180TaskHandle;
TaskHandle_t analogMeasurementsTaskHandle;

void i2c_master_init()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000
        };
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
    esp_err_t err;
    uint32_t pressure;
    uint8_t flags = 0;
    float altitude;
    float temperature;

    while(1) 
    {
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

        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) == pdTRUE) // oczekiwanie 1s
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

        if(DEBUG)
        {
            unsigned int used = get_used_stack_size(4096, bmp180TaskHandle);
            ESP_LOGI(TAG, "used size 2 BMP TASK: %d", used); // around 3600
            ESP_LOGI(TAG, "UNUSED size 2 BMP TASK: %d", 4096-used);
        }
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
    vTaskDelete( NULL );
}

void analog_measurements_task(void * params)
{
    int firstChannelResult = 0;
    int secondChannelResult = 0;

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
        firstChannelResult = convert_soil_moisture_value(firstChannelResult);

        ADS1115_request_single_ended_AIN1(); 
        while(!ADS1115_get_conversion_state()) 
        {
            vTaskDelay(pdMS_TO_TICKS(5));          
        }
        secondChannelResult = ADS1115_get_conversion();
        secondChannelResult = convert_soil_moisture_value(secondChannelResult);

        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) == pdTRUE) // oczekiwanie 1s
        {
            measurements.soilMoisture1 = firstChannelResult;
            measurements.soilMoisture2 = secondChannelResult;
            xSemaphoreGive(mutex);
        }

        if(DEBUG)
        {
            unsigned int used = get_used_stack_size(4096, analogMeasurementsTaskHandle);
            ESP_LOGI(TAG, "used size 2: %d", used); // around 3600
            ESP_LOGI(TAG, "UNUSED size 2: %d", 4096-used);
        }
        vTaskDelay(pdMS_TO_TICKS(2500));
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
    if(mutex == NULL)
    {
        ESP_LOGI(TAG, "Mutex NOT created");
        return;
    }
    ESP_LOGI(TAG, "Mutex created");

    if(pdPASS == xTaskCreate(&analog_measurements_task, "analog_measurements_task", 4096, NULL, 5, &analogMeasurementsTaskHandle))
    {
        ESP_LOGI(TAG, "Analog measurements task created");
    }

    ret = bmp180_init(SDA_PIN, SCL_PIN);
    if(ret == ESP_OK)
    {
        if(pdPASS == xTaskCreate(&bmp180_measurements_task, "bmp180_measurements_task", 4096, NULL, 5, &bmp180TaskHandle))
        {
            ESP_LOGI(TAG, "BMP180 measurements task created");
        }
    } 
    else 
    {
        ESP_LOGE(TAG, "BMP180 init failed with error = %d", ret);
        measurements.bmpPressure = -999.0;
        measurements.bmpTemperature = -999.0;
    }
}
