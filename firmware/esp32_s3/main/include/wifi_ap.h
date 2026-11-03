/**
 * @file wifi_ap.h
 * @brief Wi-Fi SoftAP initialization interface.
 *
 * Initializes the ESP32-S3 as a Wi-Fi access point for the ball-and-plate
 * system. Other devices, such as the ESP32-CAM and PC, can connect to this
 * network to exchange camera measurements, control commands, and telemetry.
 */

#ifndef WIFI_AP_H
#define WIFI_AP_H

#include "esp_err.h"

/**
 * @brief Initialize and start the Wi-Fi access point.
 *
 * Initializes NVS, the network interface layer, the default event loop,
 * and the Wi-Fi driver. Configures the ESP32 in SoftAP mode using the
 * configured SSID, password, channel, and maximum client count.
 *
 * Also registers a Wi-Fi event handler to log client connection and
 * disconnection events.
 *
 * @return
 *      - ESP_OK: Access point started successfully.
 *      - ESP_FAIL: Failed to create the AP network interface.
 *      - Other ESP error codes: An initialization or configuration step failed.
 *
 * @note This function is intended to be called once during system startup.
 *       It does not currently support repeated initialization or cleanup
 *       after a partial initialization failure.
 */
esp_err_t wifi_ap_init(void);

#endif