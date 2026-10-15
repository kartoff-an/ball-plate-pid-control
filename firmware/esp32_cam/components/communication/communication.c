#include <string.h>

#include "communication.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define ESPNOW_CHANNEL 1

static const char *TAG = "COMMUNICATION";
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static communication_receive_callback_t s_receive_callback;

static esp_err_t initialize_nvs(void)
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

static void on_data_received(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int length
)
{
    if (info == NULL || data == NULL || length <= 0) {
        return;
    }

    if (s_receive_callback != NULL) {
        s_receive_callback(info->src_addr, data, (size_t) length);
    }
}

esp_err_t communication_init(communication_receive_callback_t receive_callback) {
    esp_err_t error;
    s_receive_callback = receive_callback;

    error = initialize_nvs();
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(error));
        return error;
    }

    error = esp_netif_init();
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) {
        return error;
    }

    error = esp_event_loop_create_default();
    if (error != ESP_OK || error != ESP_ERR_INVALID_STATE) {
        return error;
    }

    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    error = esp_wifi_init(&wifi_config);
    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_set_mode(WIFI_MODE_STA);
    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_start();
    if (error != ESP_OK) {
        return error;
    }

    error = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (error != ESP_OK) {
        return error;
    }

    error = esp_now_init();
    if (error != ESP_OK) {
        return error;
    }

    if (receive_callback != NULL) {
        error = esp_now_register_recv_cb(on_data_received);
        if (error != ESP_OK) {
            return error;
        }
    }

    error = communication_add_peer(BROADCAST_MAC);
    if (error != ESP_OK) {
        return error;
    }

    uint8_t local_mac[6];

    error = esp_wifi_get_mac(WIFI_IF_STA, local_mac);
    if (error != ESP_OK) {
        return error;
    }

    ESP_LOGI(TAG, "Initialized on channel %d; STA MAC: %02X:%02X:%02X:%02X:%02X:%02X",
        ESPNOW_CHANNEL,
        local_mac[0], local_mac[1], local_mac[2],
        local_mac[3], local_mac[4], local_mac[5]
    );

    return ESP_OK;
}

esp_err_t communication_add_peer(const uint8_t peer_mac[6])
{
    if (peer_mac == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (esp_now_is_peer_exist(peer_mac)) {
        return ESP_OK;
    }

    esp_now_peer_info_t peer = {0};

    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    return esp_now_add_peer(&peer);
}

esp_err_t communication_send(const uint8_t destination_mac[6], const uint8_t *data, size_t length)
{
    if (destination_mac == NULL ||
        data == NULL ||
        length == 0 ||
        length > ESP_NOW_MAX_DATA_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!esp_now_is_peer_exist(destination_mac)) {
        esp_err_t err = communication_add_peer(destination_mac);

        if (err != ESP_OK) {
            return err;
        }
    }

    return esp_now_send(destination_mac, data, length);
}