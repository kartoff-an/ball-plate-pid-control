"""Fake camera frames and telemetry for GUI development without hardware."""

import math
import time

import numpy as np

from config import FRAME_HEIGHT, FRAME_WIDTH
from telemetry_receiver import TelemetrySample

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


def pixels_to_mm(position):
    """Image pixels to plate millimeters, with (0, 0) at the plate center."""
    x, y = position
    return (x - FRAME_WIDTH / 2) * MM_PER_PIXEL, (FRAME_HEIGHT / 2 - y) * MM_PER_PIXEL


# Mirrors the defaults in firmware/esp32_s3/main/pid_controller.c.
SERVO_NEUTRAL_US = 1500
SERVO_MIN_US = 1420
SERVO_MAX_US = 1580
MAX_CORRECTION_US = 80.0
MOCK_KP = 0.5


class MockTelemetry:
    """Builds firmware-shaped TelemetrySamples from a ball position in pixels.

    Uses the same P-only response and servo limits as the firmware so the
    numbers look realistic, with servo output reported as disabled.
    """

    def __init__(self, target=(FRAME_WIDTH / 2, FRAME_HEIGHT / 2)):
        self._target = target
        self._sequence = 0

    def sample(self, ball_px):
        ball_x, ball_y = round(ball_px[0]), round(ball_px[1])
        target_mm = pixels_to_mm(self._target)
        ball_mm = pixels_to_mm((ball_x, ball_y))
        error = (target_mm[0] - ball_mm[0], target_mm[1] - ball_mm[1])
        output = tuple(max(-MAX_CORRECTION_US, min(MAX_CORRECTION_US, MOCK_KP * e)) for e in error)
        servo = tuple(
            max(SERVO_MIN_US, min(SERVO_MAX_US, int(SERVO_NEUTRAL_US + o))) for o in output
        )

        sample = TelemetrySample(
            sequence=self._sequence,
            ball_detected=True,
            ball_x=ball_x,
            ball_y=ball_y,
            measurement_age_ms=15,
            error_x_mm=float(error[0]),
            error_y_mm=float(error[1]),
            output_x_us=output[0],
            output_y_us=output[1],
            servo_x_us=servo[0],
            servo_y_us=servo[1],
            controller_enabled=False,
            received_at=time.monotonic(),
        )
        self._sequence += 1
        return sample
