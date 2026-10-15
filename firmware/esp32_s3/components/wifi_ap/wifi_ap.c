
#include "wifi_ap.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_mac.h"

#include <stdio.h>

#define WIFI_AP_SSID       "BallPlateSystem"
#define WIFI_AP_PASSWORD   "BallPlateSystem1234"
#define WIFI_AP_CHANNEL    1
#define WIFI_AP_MAX_CLIENTS 2

static const char *TAG = "WIFI_AP";

static esp_err_t init_nvs(void)
{
    esp_err_t error = nvs_flash_init();

    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        error = nvs_flash_erase();
        if (error != ESP_OK) {
            return error;
        }

        error = nvs_flash_init();
    }

    return error;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event = event_data;
        ESP_LOGI(TAG, "Client connected: " MACSTR, MAC2STR(event->mac));
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const wifi_event_ap_stadisconnected_t *event = event_data;
        ESP_LOGW(TAG, "Client disconnected: " MACSTR, MAC2STR(event->mac));
    }
}

esp_err_t wifi_ap_init(void)
{
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

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create AP network interface");
        return ESP_FAIL;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();

    error = esp_wifi_init(&init_config);
    if (error != ESP_OK) {
        return error;
    }

    error = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler,NULL);
    if (error != ESP_OK) {
        return error;
    }

    wifi_config_t ap_config = {0};

    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s", WIFI_AP_SSID);
    snprintf((char *)ap_config.ap.password, sizeof(ap_config.ap.password), "%s", WIFI_AP_PASSWORD);

    ap_config.ap.ssid_len = sizeof(WIFI_AP_SSID) - 1;
    ap_config.ap.channel = WIFI_AP_CHANNEL;
    ap_config.ap.max_connection = WIFI_AP_MAX_CLIENTS;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    error = esp_wifi_set_mode(WIFI_MODE_AP);
    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_start();
    if (error != ESP_OK) {
        return error;
    }

    ESP_LOGI(TAG, "Access point started");
    ESP_LOGI(TAG, "SSID: %s", WIFI_AP_SSID);
    ESP_LOGI(TAG, "Channel: %d", WIFI_AP_CHANNEL);
    ESP_LOGI(TAG, "AP IP: 192.168.4.1");

    return ESP_OK;
}
