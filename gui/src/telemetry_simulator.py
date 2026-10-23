"""Sends fake ESP32-S3 telemetry packets so live_telemetry can be tried without hardware.

Packets use the same JSON format as firmware/esp32_s3/main/telemetry.c. From gui/:

    python -m telemetry_simulator                 # to this computer
    python -m telemetry_simulator --pause-every 5 # stop for 2 s now and then, to see "stale"

The GUI should be started separately with python -m live_telemetry.
"""

import argparse
import random
import socket
import time

from mock_data import MockCamera, MockTelemetry
from telemetry_receiver import TELEMETRY_PORT


def _bool(value):
    return "true" if value else "false"


def to_firmware_json(s):
    """Same layout as the snprintf() in telemetry.c, including two-decimal floats."""
    return (
        f'{{"type":"telemetry","sequence":{s.sequence},"ball_detected":{_bool(s.ball_detected)},'
        f'"ball_x":{s.ball_x},"ball_y":{s.ball_y},"measurement_age_ms":{s.measurement_age_ms},'
        f'"error_x_px":{s.error_x_px:.2f},"error_y_px":{s.error_y_px:.2f},'
        f'"output_x_us":{s.output_x_us:.2f},"output_y_us":{s.output_y_us:.2f},'
        f'"servo_x_us":{s.servo_x_us},"servo_y_us":{s.servo_y_us},'
        f'"controller_enabled":{_bool(s.controller_enabled)}}}'
    ).encode("utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1", help="destination address (default: this computer)")
    parser.add_argument("--port", type=int, default=TELEMETRY_PORT)
    parser.add_argument("--rate", type=float, default=20.0, help="packets per second (firmware max is 20)")
    parser.add_argument("--drop", type=float, default=0.0, help="fraction of packets to skip, 0-1")
    parser.add_argument("--pause-every", type=float, default=0.0, help="seconds between 2 s pauses (0 = never)")
    args = parser.parse_args()

    camera = MockCamera()
    mock = MockTelemetry()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print(f"Sending simulated telemetry to {args.host}:{args.port} at {args.rate:g} Hz (Ctrl+C to stop)")
    start = time.perf_counter()
    next_pause = args.pause_every
    try:
        while True:
            t = time.perf_counter() - start
            if args.pause_every and t >= next_pause:
                print("Pausing for 2 s")
                time.sleep(2.0)
                next_pause = time.perf_counter() - start + args.pause_every

            # Always advance the sequence so dropped packets show up as gaps.
            sample = mock.sample(camera.ball_position(t))
            if random.random() >= args.drop:
                sock.sendto(to_firmware_json(sample), (args.host, args.port))
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
