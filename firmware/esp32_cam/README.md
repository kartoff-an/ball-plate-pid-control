# ESP32-CAM Vision Firmware

This ESP-IDF project powers the vision sensor for the ball-and-plate control
system. The ESP32-CAM captures grayscale camera frames, identifies the ball by
its bright connected region, reports the result to the ESP32-S3 controller over
ESP-NOW, and streams live frames to a desktop client over UDP.

## Overview

- Captures 160 × 120 grayscale frames using the ESP32-CAM pinout.
- Tracks the largest bright connected component while enforcing configured
  area/size limits.
- Sends ball measurements (frame sequence, detection flag, center point, and
  area) to the controller board over ESP-NOW.
- Streams raw grayscale frames to a registered UDP client on port `9000`.
- Works as the companion sensor node for the firmware in `../esp32_s3`.

## Hardware and requirements

- ESP32-CAM (or equivalent board with the same camera pinout and
  PSRAM support).
- ESP-IDF installed and configured in the terminal environment.
- ESP32-S3 controller running the companion firmware in `../esp32_s3`.
- A stable Wi-Fi link between the camera and the S3 access point.

The project defaults target the `esp32` chip, enable PSRAM, and use a 4 MB
flash configuration in `sdkconfig.defaults`.

## System flow

1. The camera connects to the Wi-Fi access point created by the ESP32-S3.
2. The ESP-NOW peer is configured with the controller MAC address.
3. The camera starts its image-processing loop and detects the ball in each
   frame.
4. A ball measurement packet is transmitted to the S3 for the control loop.
5. A UDP client can register by sending `HELO` to the camera's IP and port
   `9000`, after which the camera streams `GRAY`-prefixed frame data.

## Source layout

- `main/app_main.c` — startup sequence and module initialization.
- `main/camera.c` — camera setup, capture loop, and frame processing.
- `main/ball_detection.c` — ball detection and connected-component logic.
- `main/ball_now.c` — ESP-NOW send/receive logic for controller communication.
- `main/wifi_station.c` — Wi-Fi station connection to the S3 network.
- `main/wifi_stream.c` — UDP stream server and frame transmission.
- `main/vision_tuner.c` — vision tuning / calibration helpers.
- `main/include/` — shared data structures and public headers.
- `sdkconfig.defaults` — default ESP-IDF configuration for this project.

## Connection configuration

The camera connects to the access point created by the ESP32-S3. Make sure the
SSID and password in `main/wifi_station.c` match the access point settings in
`../esp32_s3/main/wifi_ap.c`.

The ESP32-S3 MAC address is configured as `S3_AP_MAC` in `main/app_main.c`.
Update that value if the controller board has a different MAC address. Both
boards must share the same Wi-Fi channel for ESP-NOW to work reliably.

Wi-Fi credentials are currently source-level configuration; do not commit
personal or production credentials to version control.

## Build, flash, and monitor

Open an ESP-IDF-enabled terminal at the repository root, then run:

```sh
cd firmware/esp32_cam
idf.py set-target esp32
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial port for the ESP32-CAM (for example, `COM5` on
Windows or `/dev/ttyUSB0` on Linux). On a first flash, place the board into
programming mode using the required BOOT/IO0 procedure for the board or USB-to-UART
adapter. Exit the serial monitor with `Ctrl+]`.

On startup, the serial log reports the camera's assigned IP address. The desktop
preview client must send `HELO` from the UDP port it expects to receive frames on;
that registration is then used for the camera's UDP stream.

## Troubleshooting

- **Wi-Fi does not connect:** Check that the S3 access point is active and that
  the SSID/password match. The camera retries after disconnects.
- **Camera initialization fails:** Verify the board model, ribbon connection,
  power rail, and PSRAM support.
- **No ball is reported:** Use stronger lighting and a bright ball against a
  darker background. The detection thresholds and area limits are tuned in the
  source and may need adjustment.
- **Frames are not received:** Confirm that the host has joined the S3 access
  point, use the camera IP from the serial log, and ensure the client registers
  with `HELO` on UDP port `9000`.
- **ESP-NOW measurements do not reach the S3:** Check the configured S3 MAC
  address and verify that both devices are on the same Wi-Fi channel.

## Notes

This firmware is intended to be used alongside the controller project in
`../esp32_s3` and the desktop GUI in the repository root. Keep the camera and
controller configuration in sync whenever changing the Wi-Fi credentials, MAC
address, or message payload format.
