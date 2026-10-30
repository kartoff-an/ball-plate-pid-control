#include "wifi_station.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_SSID           "BallPlateSystem"
#define WIFI_PASSWORD       "BallPlateSystem1234"

#define WIFI_CONNECTED_BIT  BIT0

static const char *TAG = "WIFI_STA";

static EventGroupHandle_t wifi_event_group;
static bool is_initialized = false;

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();

        if (err != ESP_OK) {
            return err;
        }

        err = nvs_flash_init();
    }

    return err;
}

static void wifi_event_handler(void *context, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)context;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Connecting to %s", WIFI_SSID);
        esp_wifi_connect();
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

        ESP_LOGW(TAG, "Disconnected; retrying connection");
        esp_wifi_connect();
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;

        ESP_LOGI(TAG, "Connected to %s", WIFI_SSID);
        ESP_LOGI(TAG, "CAM IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Gateway IP: " IPSTR, IP2STR(&event->ip_info.gw));

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_station_connect(void)
{
    if (is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = init_nvs();

    if (error != ESP_OK) {
        return error;
    }

    error = esp_netif_init();

    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
        return error;
    }

    error = esp_event_loop_create_default();

    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
        return error;
    }

    if (esp_netif_create_default_wifi_sta() == NULL) {
        ESP_LOGE(TAG, "Failed to create station network interface");
        return ESP_FAIL;
    }

    wifi_event_group = xEventGroupCreate();

    if (wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();

    error = esp_wifi_init(&wifi_config);

    if (error != ESP_OK) {
        return error;
    }

    error = esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_handler,
        NULL
    );

    if (error != ESP_OK) {
        return error;
    }

    error = esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi_event_handler,
        NULL
    );

    if (error != ESP_OK) {
        return error;
    }

    wifi_config_t station_config = {0};

    snprintf(
        (char *)station_config.sta.ssid,
        sizeof(station_config.sta.ssid),
        "%s",
        WIFI_SSID
    );
    snprintf(
        (char *)station_config.sta.password,
        sizeof(station_config.sta.password),
        "%s",
        WIFI_PASSWORD
    );

    station_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    error = esp_wifi_set_mode(WIFI_MODE_STA);

    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_set_config(WIFI_IF_STA, &station_config);

    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_start();

    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_set_ps(WIFI_PS_NONE);
    if (error != ESP_OK) {
        ESP_LOGW(TAG, "Could not disable Wi-Fi power saving");
    }

    is_initialized = true;

    ESP_LOGI(TAG, "Waiting for access point connection");

    EventBits_t event_bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    return (event_bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}
