/**
 * @file ball_measurement.h
 * @brief Shared data format for ball measurements exchanged between devices.
 *
 * Defines the measurement packet transmitted from the ESP32-CAM to the
 * ESP32-S3. The packet contains ball detection status, frame sequence,
 * detected center coordinates, and component area.
 *
 * The structure is packed to prevent compiler-inserted padding and
 * maintain a consistent binary packet layout.
 *
 * @note Both devices must use the same field definitions and compatible
 *       byte ordering when transmitting and interpreting this structure.
 */

#ifndef BALL_MEASUREMENT_H
#define BALL_MEASUREMENT_H

#include <stdint.h>

#define BALL_MEASUREMENT_MAGIC      0xBA11
#define BALL_MEASUREMENT_VERSION    1

/**
 * @brief Ball detection measurement packet.
 *
 * Contains the result of one camera frame's ball detection process.
 *
 * | Field | Type | Size | Description |
 * |---|---|---:|---|
 * | magic | uint16_t | 2 bytes | Packet identifier |
 * | version | uint8_t | 1 byte | Packet format version |
 * | detected | uint8_t | 1 byte | Ball detection status |
 * | sequence | uint32_t | 4 bytes | Camera frame sequence number |
 * | center_x | uint16_t | 2 bytes | Detected horizontal center coordinate |
 * | center_y | uint16_t | 2 bytes | Detected vertical center coordinate |
 * | area | uint32_t | 4 bytes | Detected component area in pixels |
 *
 * Total packed size: 16 bytes.
 *
 * @note The structure uses packed layout to ensure a fixed 16-byte size.
 * @note Coordinates are expressed in the camera's image coordinate system.
 * @note The meaning and units of the coordinates depend on the camera
 *       crop and coordinate-offset configuration.
 * @note When detected is zero, center_x, center_y, and area should not
 *       be treated as valid ball measurements.
 */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t detected;
    uint32_t sequence;
    uint16_t center_x;
    uint16_t center_y;
    uint32_t area;
} ball_measurement_t;

#endif /* BALL_MEASUREMENT_H*/
