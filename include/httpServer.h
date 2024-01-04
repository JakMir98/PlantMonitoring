#ifndef HTTPSERVER_H
#define HTTPSERVER_H

#include "driver/gpio.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include <esp_http_server.h>
#include "freertos/event_groups.h"

#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>
#include "constants.h"

#define WIFI_SSID ""
#define WIFI_PASS ""
#define MAXIMUM_RETRY 5
#define STATIC_IP_ADDR        "192.168.0.66"
#define STATIC_NETMASK_ADDR   "255.255.255.0"
#define STATIC_GW_ADDR        "192.168.0.1"
#define MAIN_DNS_SERVER       "192.168.0.1"
#define BACKUP_DNS_SERVER     "0.0.0.0"

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

extern Measurements measurements;

void connect_wifi(void);
esp_err_t send_web_page(httpd_req_t *req);
esp_err_t get_req_handler(httpd_req_t *req);
esp_err_t relay_on_handler(httpd_req_t *req);
esp_err_t relay_off_handler(httpd_req_t *req);
httpd_handle_t setup_server(void);

void stop_webserver(httpd_handle_t server);

#endif