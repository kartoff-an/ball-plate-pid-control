import json
import math
import socket

import config

class ESPSender:
    """Sends commands to the ESP32-S3 over UDP."""

    def __init__(self) -> None:
        self.address = (config.ESP32_HOST, config.ESP32_COMMAND_PORT)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, message: dict) -> None:
        payload = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._socket.sendto(payload, self.address)

    def set_target(self, x: float, y: float) -> None:
        """Set the desired ball position"""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Target coordinates must be finite.")

        self.send({
            "type": "target",
            "x": x,
            "y": y
        })

    def set_pid_gains(
            self,
            kp_x: float, ki_x: float, kd_x: float,
            kp_y: float, ki_y: float, kd_y: float,
    ) -> None:
        gains = (kp_x, ki_x, kd_x, kp_y, ki_y, kd_y)

        if not all(math.isfinite(value) for value in gains):
            raise ValueError("PID gains must be finite numbers.")

        if any(value < 0 for value in gains):
            raise ValueError("PID gains must not be negative.")

        self.send({
            "type": "pid_config",
            "axis_x": {
                "kp": kp_x,
                "ki": ki_x,
                "kd":kd_x
            },
            "axis_y": {
                "kp": kp_y,
                "ki": ki_y,
                "kd": kd_y
            }
        })

    def start(self) -> None:
        """Request that the controller starts."""
        self.send({
            "type": "control",
            "command": "start"
        })

    def stop(self) -> None:
            """Request that the controller stops."""
            self.send({
                "type": "control",
                "command": "stop"
            })

    def reset_pid(self) -> None:
        """Request a reset of PID controller state."""
        self.send({
            "type": "control",
            "command": "reset_pid",
        })

    def close(self) -> None:
        """Release the UDP socket."""
        self._socket.close()