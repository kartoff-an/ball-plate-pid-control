#ifndef BALL_MEASUREMENT_H
#define BALL_MEASUREMENT_H

#include <stdint.h>

#define BALL_MEASUREMENT_MAGIC      0xBA11
#define BALL_MEASUREMENT_VERSION    1

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t detected;
    uint32_t sequence;
    uint16_t center_x;
    uint16_t center_y;
    uint32_t area;
} ball_measurement_t;

#endif
