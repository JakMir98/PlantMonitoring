#include <stdio.h>
#include <stdlib.h>
#include <string.h> //Requires by memset

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "esp_adc_cal.h"

#include "driver/gpio.h"
#include <driver/adc.h>

#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>

#include "nvs_flash.h"

#include "driver/i2c.h"
#include "bmp180.h"
#include "DHT.h"

typedef struct Measurements
{
    float dht_temperature;
    float dht_humidity;
    float soilMoisture1;
    float soilMoisture2;
    float bmp_pressure;
    float bmp_temperature
} Measurements;

const int LED_PIN = 2;
const int POT_PIN = 15;
const int PUSH_BUTTON_PIN = 4;
const TickType_t oneMsDelay = 1 / portTICK_PERIOD_MS;
const TickType_t fiveHundredMsDelay = 500 / portTICK_PERIOD_MS;
const TickType_t oneSecDelay = 1000 / portTICK_PERIOD_MS;

#define EXAMPLE_ESP_WIFI_SSID "UPC4707825"
#define EXAMPLE_ESP_WIFI_PASS "Rk8jwm5dssne"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define EXAMPLE_STATIC_IP_ADDR        "192.168.0.66"
#define EXAMPLE_STATIC_NETMASK_ADDR   "255.255.255.0"
#define EXAMPLE_STATIC_GW_ADDR        "192.168.0.1"
#define EXAMPLE_MAIN_DNS_SERVER       "8.8.8.8"
#define EXAMPLE_BACKUP_DNS_SERVER     "8.8.4.4"

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

char test_on_resp[]  = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"10\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">  <link rel=\"icon\" href=\"data:,\">  <link rel=\"stylesheet\" href=\"https://use.fontawesome.com/releases/v5.7.2/css/all.css\"    integrity=\"sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr\" crossorigin=\"anonymous\"> <style>html {  font-family: Arial;  display: inline-block;  margin: 0px auto;  text-align: center;}  .topnav { overflow: hidden; background-color: #241d4b; color: white; font-size: 1.7rem; } .button {  display: inline-block;  background-color: #b30000; border: none;  border-radius: 4px;  color: white;  padding: 16px 40px;  text-decoration: none;  font-size: 30px;  margin: 2px;  cursor: pointer;} .button2 {  background-color: #364cf4; } .content { padding: 20px; } .card {  background-color: white;  box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);} .card-title {  font-size: 1.2rem;  font-weight: bold;  color: #034078} .cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); } .card.temperature { color: #0e7c7b; } .card.humidity { color: #17bebb; } .card.pressure { color: #d62246; } .card.soil { color: #3fca6b; }</style>  <title>ESP32 WEB SERVER</title></head><body>  <div class=\"topnav\"><h2>ESP32 WEB SERVER</h2></div>  <div class=\"content\">    <div class=\"cards\"> <div class=\"card temperature\"><h4> Temperatura DHT: </h4><p>%.2f&deg;C</p></div> <div class=\"card humidity\"><h4> Wilgotnosc DHT: </h4><p>%.2f &percnt;</p></div> <div class=\"card temperature\"><h4> Temperatura BMP: </h4><p>%.2f&deg;C</p></div> <div class=\"card pressure\"><h4> Cisnienie BMP: </h4><p>%.2fhPa</p></div> <div class=\"card soil\"><h4> Wilgotnosc gleby 1: </h4><p>%.2f &percnt;</p></div> <div class=\"card soil\"><h4> Wilgotnosc gleby 2: </h4><p>%.2f &percnt;</p></div>  </div> <p><p><p>GPIO state: <strong> ON</strong>  </p>                 <a href=\"/led2on\"><button class=\"button\">ON</button></a>          <a href=\"/led2off\"><button class=\"button button2\">OFF</button></a>        </p>            </div></body></html>";
char test_off_resp[] = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"10\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">  <link rel=\"icon\" href=\"data:,\">  <link rel=\"stylesheet\" href=\"https://use.fontawesome.com/releases/v5.7.2/css/all.css\"    integrity=\"sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr\" crossorigin=\"anonymous\"> <style>html {  font-family: Arial;  display: inline-block;  margin: 0px auto;  text-align: center;}  .topnav { overflow: hidden; background-color: #241d4b; color: white; font-size: 1.7rem; } .button {  display: inline-block;  background-color: #b30000; border: none;  border-radius: 4px;  color: white;  padding: 16px 40px;  text-decoration: none;  font-size: 30px;  margin: 2px;  cursor: pointer;} .button2 {  background-color: #364cf4; } .content { padding: 20px; } .card {  background-color: white;  box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);} .card-title {  font-size: 1.2rem;  font-weight: bold;  color: #034078} .cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); } .card.temperature { color: #0e7c7b; } .card.humidity { color: #17bebb; } .card.pressure { color: #d62246; } .card.soil { color: #3fca6b; }</style>  <title>ESP32 WEB SERVER</title></head><body>  <div class=\"topnav\"><h2>ESP32 WEB SERVER</h2></div>  <div class=\"content\">    <div class=\"cards\"> <div class=\"card temperature\"><h4> Temperatura DHT: </h4><p>%.2f&deg;C</p></div> <div class=\"card humidity\"><h4> Wilgotnosc DHT: </h4><p>%.2f &percnt;</p></div> <div class=\"card temperature\"><h4> Temperatura BMP: </h4><p>%.2f&deg;C</p></div> <div class=\"card pressure\"><h4> Cisnienie BMP: </h4><p>%.2fhPa</p></div> <div class=\"card soil\"><h4> Wilgotnosc gleby 1: </h4><p>%.2f &percnt;</p></div> <div class=\"card soil\"><h4> Wilgotnosc gleby 2: </h4><p>%.2f &percnt;</p></div>  </div> <p><p><p>GPIO state: <strong> OFF</strong> </p>                 <a href=\"/led2on\"><button class=\"button\">ON</button></a>          <a href=\"/led2off\"><button class=\"button button2\">OFF</button></a>        </p>            </div></body></html>";

static const char *TAG = "JMBW_ESP32"; // TAG for debug
static const char *TAG_BME280 = "JMBW_BME280";

int led_state = 0;
static Measurements measurements = 
{
    .bmp_pressure = 1000.0,
    .bmp_temperature = 23.4,
    .dht_humidity = 40.6,
    .dht_temperature = 24.2,
    .soilMoisture1 = 15.1,
    .soilMoisture2 = 16.6
};

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
int wifi_connect_status = 0;
static SemaphoreHandle_t mutex;

void DHT_readings()
{
	setDHTgpio(GPIO_NUM_27);
	
	int ret = readDHT();
		
	errorHandler(ret);
    xSemaphoreTake(mutex, portMAX_DELAY);
    measurements.dht_temperature = getTemperature();
    measurements.dht_humidity = getHumidity();
    xSemaphoreGive(mutex);

    vTaskDelay(2000 / portTICK_RATE_MS);
		
}

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

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;

    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, cnt, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = SUCCESS;
    }
    else
    {
        iError = FAIL;
    }
    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError = BME280_INIT_VALUE;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (cnt > 1)
    {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK)
    {
        iError = SUCCESS;
    }
    else
    {
        iError = FAIL;
    }

    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

void get_bme280_readings()
{
    struct bme280_t bme280 = {
        .bus_write = BME280_I2C_bus_write,
        .bus_read = BME280_I2C_bus_read,
        .dev_addr = BME280_I2C_ADDRESS1,
        .delay_msec = BME280_delay_msek};

    s32 com_rslt;
    s32 v_uncomp_pressure_s32;
    s32 v_uncomp_temperature_s32;
    s32 v_uncomp_humidity_s32;

    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

    com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);

    bme280_read_uncomp_pressure_temperature_humidity(&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

    ESP_LOGI(TAG_BME280, "%.2f degC / %.2f hPa / %.2f %%",
             bme280_compensate_temperature_double(v_uncomp_temperature_s32),
             bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100, // Pa -> hPa
             bme280_compensate_humidity_double(v_uncomp_humidity_s32));

    xSemaphoreTake(mutex, portMAX_DELAY);
	measurements.bmp_temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
    measurements.bmp_pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100;
    xSemaphoreGive(mutex);
}

static esp_err_t set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

static void set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp client");
        return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(EXAMPLE_STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) 
    {
        set_static_ip(arg);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        wifi_connect_status = 0;
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "static ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        wifi_connect_status = 1;
    }
}

void connect_wifi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s",
                 EXAMPLE_ESP_WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s",
                 EXAMPLE_ESP_WIFI_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));

    vEventGroupDelete(s_wifi_event_group);
}

esp_err_t send_web_page(httpd_req_t *req)
{
    int response;
    if (led_state == 0)
    {
        char response_data[sizeof(test_off_resp) + 50];
        memset(response_data, 0, sizeof(response_data));
        sprintf(response_data, test_off_resp, 
            measurements.dht_temperature, measurements.dht_humidity, 
            measurements.bmp_temperature, measurements.bmp_pressure, 
            measurements.soilMoisture1, measurements.soilMoisture2);

        response = httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
    }
        
    else
    {
        char response_data[sizeof(test_on_resp) + 50];
        memset(response_data, 0, sizeof(response_data));
        sprintf(response_data, test_on_resp, 
            measurements.dht_temperature, measurements.dht_humidity, 
            measurements.bmp_temperature, measurements.bmp_pressure, 
            measurements.soilMoisture1, measurements.soilMoisture2);
        
        response = httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
    }
        
    return response;
}
esp_err_t get_req_handler(httpd_req_t *req)
{
    return send_web_page(req);
}

esp_err_t led_on_handler(httpd_req_t *req)
{
    gpio_set_level(LED_PIN, 1);
    led_state = 1;
    return send_web_page(req);
}

esp_err_t led_off_handler(httpd_req_t *req)
{
    gpio_set_level(LED_PIN, 0);
    led_state = 0;
    return send_web_page(req);
}

httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_req_handler,
    .user_ctx = NULL};

httpd_uri_t uri_on = {
    .uri = "/led2on",
    .method = HTTP_GET,
    .handler = led_on_handler,
    .user_ctx = NULL};

httpd_uri_t uri_off = {
    .uri = "/led2off",
    .method = HTTP_GET,
    .handler = led_off_handler,
    .user_ctx = NULL};

httpd_handle_t setup_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_on);
        httpd_register_uri_handler(server, &uri_off);
    }

    return server;
}

void stop_webserver(httpd_handle_t server)
{
    if (server) {
        // Stop the httpd server 
        httpd_stop(server);
    }
}

void measurementsTask(void * parameter)
{
    esp_adc_cal_characteristics_t adc1_chars;

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);

    while(1)
    {
        float sum = 0;
        for (int i =0; i <10; i++)
        {
            sum += adc1_get_raw(ADC1_CHANNEL_0);
        }
        sum /= 10;
        ESP_LOGI(TAG, "POT = %f\n", sum);
        xSemaphoreTake(mutex, portMAX_DELAY);
        measurements.soilMoisture1 = sum;
        xSemaphoreGive(mutex);
        
        vTaskDelay(oneSecDelay);
    }
  vTaskDelete( NULL );
}

void testMeasureTask(void * params)
{
    for(;;)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        measurements.bmp_pressure += 16;
        measurements.bmp_temperature += 16;
        measurements.dht_humidity += 16;
        measurements.dht_temperature += 16;
        measurements.soilMoisture2 += 16;

        if(measurements.bmp_pressure > 10000)
        {
            measurements.bmp_pressure = 0;
            measurements.bmp_temperature = 0;
            measurements.dht_humidity = 0;
            measurements.dht_temperature = 0;
            measurements.soilMoisture2 = 0;
        }
        xSemaphoreGive(mutex);
        ESP_LOGI(TAG, "CHANGE MEASUREMENT\n");
        vTaskDelay(oneSecDelay);
    }
    vTaskDelete( NULL );
}

void buttonTask(void * parameter)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);   
    gpio_set_direction(PUSH_BUTTON_PIN, GPIO_MODE_INPUT);

    while(1) 
    {       
        if (gpio_get_level(PUSH_BUTTON_PIN) == 1)
        {  
            gpio_set_level(LED_PIN, 1);        
        } 
        else
        {
            gpio_set_level(LED_PIN, 0);        
        }

        vTaskDelay(1);
    }
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

    // GPIO initialization
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    led_state = 0;
    ESP_LOGI(TAG, "LED Control Web Server is running ... ...\n");
    setup_server();

    mutex = xSemaphoreCreateMutex();

    xTaskCreate(&measurementsTask, "Measure task", 4096, NULL, 5, NULL);
    xTaskCreate(&testMeasureTask, "second task", 4096, NULL, 5, NULL);
}
