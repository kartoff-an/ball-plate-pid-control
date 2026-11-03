/**
 * @file wifi_stream.h
 * @brief UDP streaming interface for grayscale camera frames.
 *
 * Provides functions to initialize the Wi-Fi frame streaming service
 * and submit captured grayscale frames for asynchronous transmission
 * to a registered PC.
 *
 * Frames are transmitted over UDP using indexed packets. The PC
 * registers as a receiver by sending the four-byte "HELO" message
 * to the camera on UDP port 9000.
 *
 * @note The streaming module maintains its own frame buffers. Submitted
 *       frame data is copied before wifi_stream_submit_frame() returns,
 *       so the caller may safely reuse its source buffer afterward.
 */

#ifndef WIFI_STREAM_H
#define WIFI_STREAM_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "ball_detection.h"

/**
 * @brief Initialize the UDP frame streaming service.
 *
 * Allocates the internal frame buffers, creates the frame queues,
 * and starts the UDP server task.
 *
 * The server waits for a PC registration message before transmitting
 * frames submitted through wifi_stream_submit_frame().
 *
 * @return
 * - ESP_OK: Queues, buffers, and server task initialized successfully.
 * - ESP_ERR_NO_MEM: Queue, buffer, or task allocation failed.
 * - Other error codes: Frame queue initialization failed.
 *
 * @note Call this function once after the network interface is ready.
 * @note The UDP server listens on port 9000.
 * @note This function does not wait for the server socket to bind
 *       successfully; socket initialization occurs inside the task.
 */
esp_err_t wifi_stream_init(void);

/**
 * @brief Submit a grayscale frame for asynchronous UDP transmission.
 *
 * Copies the supplied frame into an available internal buffer and
 * queues it for transmission by the streaming task.
 *
 * The streaming task prioritizes the newest queued frame. Older
 * queued frames may be discarded when newer frames are available.
 *
 * @param[in] frame_sequence Sequence number identifying the frame.
 * @param[in] grayscale_pixels Pointer to the grayscale frame data.
 * @param[in] frame_size_bytes Size of the frame buffer in bytes.
 *
 * @return
 * - ESP_OK: Frame copied and queued successfully.
 * - ESP_ERR_INVALID_ARG: Null frame pointer or incorrect frame size.
 * - ESP_ERR_INVALID_STATE: Streaming queues have not been initialized.
 * - ESP_ERR_TIMEOUT: No frame buffer is currently available.
 * - ESP_FAIL: Frame could not be added to the transmission queue.
 *
 * @note The expected frame size is CAMERA_FRAME_SIZE bytes.
 * @note The input frame is copied before this function returns.
 * @note A successful return indicates that the frame was queued,
 *       not that it was successfully delivered to the PC.
 */
esp_err_t wifi_stream_submit_frame(
    uint32_t frame_sequence,
    const uint8_t *grayscale_pixels,
    size_t frame_size_bytes
);

#endif /* WIFI_STREAM_H*/