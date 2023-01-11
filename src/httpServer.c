#include "httpServer.h"

static const char *HTTP_SERVER_TAG = "HTTP_SERVER_JMBW_ESP32"; // HTTP_SERVER_TAG for debug
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int retryNum = 0;
int wifiConnectStatus = 0;
int relayState = 0;
const int RELAY_PIN = 2;
char on_resp[]  = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"10\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">  <link rel=\"icon\" href=\"data:,\">  <link rel=\"stylesheet\" href=\"https://use.fontawesome.com/releases/v5.7.2/css/all.css\"    integrity=\"sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr\" crossorigin=\"anonymous\"> <style>html {  font-family: Arial;  display: inline-block;  margin: 0px auto;  text-align: center;}  .topnav { overflow: hidden; background-color: #241d4b; color: white; font-size: 1.7rem; } .button {  display: inline-block;  background-color: #b30000; border: none;  border-radius: 4px;  color: white;  padding: 16px 40px;  text-decoration: none;  font-size: 30px;  margin: 2px;  cursor: pointer;} .button2 {  background-color: #364cf4; } .content { padding: 20px; } .card {  background-color: white;  box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);} .card-title {  font-size: 1.2rem;  font-weight: bold;  color: #034078} .cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); } .card.temperature { color: #0e7c7b; } .card.humidity { color: #17bebb; } .card.pressure { color: #d62246; } .card.soil { color: #3fca6b; }</style>  <title>JMBW ESP32 WEB SERVER</title></head><body>  <div class=\"topnav\"><h2>JMBW ESP32 WEB SERVER</h2></div>  <div class=\"content\">    <div class=\"cards\"> <div class=\"card humidity\"><h4> Wysokosc BMP: </h4><p>%.2f m</p></div> <div class=\"card temperature\"><h4> Temperatura BMP: </h4><p>%.2f&deg;C</p></div> <div class=\"card pressure\"><h4> Cisnienie BMP: </h4><p>%.2fPa</p></div> <div class=\"card soil\"><h4> Wilgotnosc gleby 1: </h4><p>%.2f &percnt;</p></div> <div class=\"card soil\"><h4> Wilgotnosc gleby 2: </h4><p>%.2f &percnt;</p></div>  </div> <p><p><p>RELAY state: <strong> ON</strong>  </p>                 <a href=\"/relayon\"><button class=\"button\">ON</button></a>          <a href=\"/relayoff\"><button class=\"button button2\">OFF</button></a>        </p>            </div></body></html>";
char off_resp[] = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"10\"> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">  <link rel=\"icon\" href=\"data:,\">  <link rel=\"stylesheet\" href=\"https://use.fontawesome.com/releases/v5.7.2/css/all.css\"    integrity=\"sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr\" crossorigin=\"anonymous\"> <style>html {  font-family: Arial;  display: inline-block;  margin: 0px auto;  text-align: center;}  .topnav { overflow: hidden; background-color: #241d4b; color: white; font-size: 1.7rem; } .button {  display: inline-block;  background-color: #b30000; border: none;  border-radius: 4px;  color: white;  padding: 16px 40px;  text-decoration: none;  font-size: 30px;  margin: 2px;  cursor: pointer;} .button2 {  background-color: #364cf4; } .content { padding: 20px; } .card {  background-color: white;  box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);} .card-title {  font-size: 1.2rem;  font-weight: bold;  color: #034078} .cards { max-width: 700px; margin: 0 auto; display: grid; grid-gap: 2rem; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); } .card.temperature { color: #0e7c7b; } .card.humidity { color: #17bebb; } .card.pressure { color: #d62246; } .card.soil { color: #3fca6b; }</style>  <title>JMBW ESP32 WEB SERVER</title></head><body>  <div class=\"topnav\"><h2>JMBW ESP32 WEB SERVER</h2></div>  <div class=\"content\">    <div class=\"cards\"> <div class=\"card humidity\"><h4> Wysokosc BMP: </h4><p>%.2f m</p></div> <div class=\"card temperature\"><h4> Temperatura BMP: </h4><p>%.2f&deg;C</p></div> <div class=\"card pressure\"><h4> Cisnienie BMP: </h4><p>%.2fPa</p></div> <div class=\"card soil\"><h4> Wilgotnosc gleby 1: </h4><p>%.2f &percnt;</p></div> <div class=\"card soil\"><h4> Wilgotnosc gleby 2: </h4><p>%.2f &percnt;</p></div>  </div> <p><p><p>RELAY state: <strong> OFF</strong> </p>                 <a href=\"/relayon\"><button class=\"button\">ON</button></a>          <a href=\"/relayoff\"><button class=\"button button2\">OFF</button></a>        </p>            </div></body></html>";

extern Measurements measurements;

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
        ESP_LOGE(HTTP_SERVER_TAG, "Failed to stop dhcp client");
        return;
    }
    
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(HTTP_SERVER_TAG, "Failed to set ip info");
        return;
    }
    ESP_LOGD(HTTP_SERVER_TAG, "Success to set static ip: %s, netmask: %s, gw: %s", STATIC_IP_ADDR, STATIC_NETMASK_ADDR, STATIC_GW_ADDR);
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(set_dns_server(netif, ipaddr_addr(BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
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
        if (retryNum < MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            retryNum++;
            ESP_LOGI(HTTP_SERVER_TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        wifiConnectStatus = 0;
        ESP_LOGI(HTTP_SERVER_TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(HTTP_SERVER_TAG, "static ip:" IPSTR, IP2STR(&event->ip_info.ip));
        retryNum = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        wifiConnectStatus = 1;
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
                                                        sta_netif,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        sta_netif,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(HTTP_SERVER_TAG, "wifi_init_sta finished.");

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
        ESP_LOGI(HTTP_SERVER_TAG, "connected to ap SSID:%s",
                 WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(HTTP_SERVER_TAG, "Failed to connect to SSID:%s",
                 WIFI_SSID);
    }
    else
    {
        ESP_LOGE(HTTP_SERVER_TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));

    vEventGroupDelete(s_wifi_event_group);
}

esp_err_t send_web_page(httpd_req_t *req)
{
    int response;
    if (relayState == 0)
    {
        char response_data[sizeof(off_resp) + 50];
        memset(response_data, 0, sizeof(response_data));
        sprintf(response_data, off_resp, 
            measurements.bmpAltitude, 
            measurements.bmpTemperature, measurements.bmpPressure, 
            measurements.soilMoisture1, measurements.soilMoisture2);

        response = httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
    }
        
    else
    {
        char response_data[sizeof(on_resp) + 50];
        memset(response_data, 0, sizeof(response_data));
        sprintf(response_data, on_resp, 
            measurements.bmpAltitude, 
            measurements.bmpTemperature, measurements.bmpPressure, 
            measurements.soilMoisture1, measurements.soilMoisture2);
        
        response = httpd_resp_send(req, response_data, HTTPD_RESP_USE_STRLEN);
    }
        
    return response;
}

esp_err_t get_req_handler(httpd_req_t *req)
{
    return send_web_page(req);
}

esp_err_t relay_on_handler(httpd_req_t *req)
{
    gpio_set_level(RELAY_PIN, 1);
    relayState = 1;
    return send_web_page(req);
}

esp_err_t relay_off_handler(httpd_req_t *req)
{
    gpio_set_level(RELAY_PIN, 0);
    relayState = 0;
    return send_web_page(req);
}

httpd_uri_t uri_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_req_handler,
    .user_ctx = NULL};

httpd_uri_t uri_on = {
    .uri = "/relayon",
    .method = HTTP_GET,
    .handler = relay_on_handler,
    .user_ctx = NULL};

httpd_uri_t uri_off = {
    .uri = "/relayoff",
    .method = HTTP_GET,
    .handler = relay_off_handler,
    .user_ctx = NULL};

httpd_handle_t setup_server(void)
{
    // GPIO initialization
    gpio_pad_select_gpio(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);

    relayState = 0;

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
