/**
 * @file wifi_station.h
 * @brief Wi-Fi station connection interface.
 *
 * Provides the initialization and connection interface for the
 * ESP32 station (STA) network mode.
 *
 * The station connects to the configured access point and waits
 * until an IP address is assigned through DHCP.
 */

#ifndef WIFI_STATION_H
#define WIFI_STATION_H

#include "esp_err.h"

/**
 * @brief Initialize Wi-Fi and connect to the configured access point.
 *
 * Initializes NVS, the network interface, the default event loop,
 * and the Wi-Fi station driver. Registers Wi-Fi and IP event handlers,
 * configures the station credentials, and starts the Wi-Fi connection.
 *
 * The function blocks until the station obtains an IP address.
 *
 * @return
 * - ESP_OK: Successfully connected and obtained an IP address.
 * - ESP_ERR_INVALID_STATE: The station module has already been initialized.
 * - ESP_ERR_NO_MEM: Event group allocation failed.
 * - ESP_FAIL: Station network interface creation failed.
 * - Other ESP-IDF error codes: NVS, network interface, Wi-Fi driver,
 *   event registration, or configuration initialization failed.
 *
 * @note Connection retries are handled automatically after
 *       disconnection events.
 * @note Wi-Fi power saving is disabled after Wi-Fi startup.
 * @note This function can block indefinitely if the access point
 *       is unavailable or the station cannot obtain an IP address.
 * @note Intended to be called once during system startup.
 */
esp_err_t wifi_station_connect(void);

#endif
