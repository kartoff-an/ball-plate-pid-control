#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <esp_err.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*communication_receive_callback_t)(
    const uint8_t *source_mac,
    const uint8_t *data,
    size_t length
);

esp_err_t communication_init(communication_receive_callback_t receive_callback);

esp_err_t communication_add_peer(const uint8_t peer_max[6]);

esp_err_t communication_send(
    const uint8_t destination_mac[6],
    const uint8_t *data,
    size_t length
);

#endif