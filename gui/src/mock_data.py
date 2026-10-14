"""Fake camera frames for GUI development until the ESP32-CAM feed is integrated."""

import math

import numpy as np

# Matches the ESP32-CAM stream (QQVGA grayscale).
FRAME_WIDTH = 160
FRAME_HEIGHT = 120

PLATE_LEVEL = 70
BALL_LEVEL = 230
BALL_RADIUS = 8
NOISE_LEVEL = 6


class MockCamera:
    """Produces frames of a bright ball moving in a slow loop over a dark plate."""

    def __init__(self, seed=0):
        self._rng = np.random.default_rng(seed)
        self._rows, self._cols = np.mgrid[0:FRAME_HEIGHT, 0:FRAME_WIDTH]

    def ball_position(self, t):
        """Ball center in image pixels at time t (seconds)."""
        x = FRAME_WIDTH / 2 + 45 * math.cos(0.6 * t)
        y = FRAME_HEIGHT / 2 + 30 * math.sin(1.2 * t)
        return x, y

    def frame(self, t):
        """Grayscale uint8 frame of shape (FRAME_HEIGHT, FRAME_WIDTH)."""
        x, y = self.ball_position(t)
        image = np.full((FRAME_HEIGHT, FRAME_WIDTH), PLATE_LEVEL, dtype=np.int16)

        inside = (self._cols - x) ** 2 + (self._rows - y) ** 2 <= BALL_RADIUS**2
        image[inside] = BALL_LEVEL

        image += self._rng.integers(-NOISE_LEVEL, NOISE_LEVEL + 1, image.shape, dtype=np.int16)
        return np.clip(image, 0, 255).astype(np.uint8)


# Placeholder scale until the camera is calibrated against the real plate.
MM_PER_PIXEL = 1.5
MAX_SERVO_ANGLE = 15.0
SERVO_GAIN = 0.2


def pixels_to_mm(position):
    """Image pixels to plate millimeters, with (0, 0) at the plate center."""
    x, y = position
    return (x - FRAME_WIDTH / 2) * MM_PER_PIXEL, (FRAME_HEIGHT / 2 - y) * MM_PER_PIXEL


def mock_servo_angles(measured, target):
    """Rough proportional response so the servo readouts move plausibly."""
    return tuple(
        max(-MAX_SERVO_ANGLE, min(MAX_SERVO_ANGLE, SERVO_GAIN * (t - m)))
        for m, t in zip(measured, target)
    )
