# ESP32-CAM Firmware

This ESP-IDF project runs on an AI Thinker ESP32-CAM. It captures low-resolution
grayscale images, finds the largest bright connected region as the ball, sends
the measurement to the ESP32-S3 over ESP-NOW, and streams frames over UDP for
the desktop preview.

## What it does

- Captures 160 × 120 grayscale frames using the AI Thinker camera pinout.
- Detects bright regions with a grayscale threshold and selects the largest
  connected component within the configured area limits.
- Sends each detection result (frame sequence, detected flag, center, and area)
  to the ESP32-S3 using ESP-NOW.
- Streams grayscale frames to a registered UDP client on port `9000`. The client
  registers by sending the four bytes `HELO` to the camera's IP and port `9000`.
  Frame packets start with `GRAY` and include the sequence number, frame size,
  packet index, and packet count.

The camera configuration and detection threshold/area limits are currently
defined in `main/camera_capture.c` and `main/app_main.c`.

## Requirements

- An AI Thinker ESP32-CAM (or compatible board with the same camera pinout and
  PSRAM).
- ESP-IDF installed and initialized in the terminal environment.
- An ESP32-S3 running the companion firmware in `../esp32_s3`.

The default project configuration targets ESP32, enables PSRAM, and selects a
4 MB flash size in `sdkconfig.defaults`.

## Configure the connection

The ESP32-CAM connects to the access point started by the ESP32-S3. Ensure the
SSID and password in `main/wifi_station.c` match the S3 access point settings
in `../esp32_s3/main/wifi_ap.c`. Change the values in both projects when using
different credentials.

The destination ESP32-S3 station MAC address is configured as `S3_AP_MAC` in
`main/app_main.c`. Update it if the S3 board's MAC address differs. Both boards
must use the same Wi-Fi channel for ESP-NOW communication.

Wi-Fi credentials are currently source-level settings; avoid committing
personal or production credentials.

## Build, flash, and monitor

Open an ESP-IDF-enabled terminal at the repository root, then run:

```sh
cd firmware/esp32_cam
idf.py set-target esp32
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial port for the ESP32-CAM (for example, `COM5` on
Windows or `/dev/ttyUSB0` on Linux). For a first flash, put the board into
download mode using its boot/IO0 control as required by the board or USB-to-UART
adapter. Exit the serial monitor with `Ctrl+]`.

On startup, the serial log reports the camera's assigned IP address. The
desktop preview client must send `HELO` from the UDP port on which it expects to
receive frames; the camera streams to that registered address and port.

## Troubleshooting

- **Wi-Fi does not connect:** Check that the S3 access point is running and that
  the SSID/password match. The camera retries after disconnects.
- **Camera initialization fails:** Check the board's camera model/pinout,
  camera ribbon connection, power, and PSRAM support.
- **No ball is reported:** Use adequate lighting and a bright ball against a
  darker background. Detection uses a fixed grayscale threshold and area range,
  so tune the values in `main/app_main.c` for the scene.
- **Frames are not received:** Confirm the host joined the S3 access point, use
  the camera IP from the serial log, and ensure the client registers with `HELO`
  on UDP port `9000`.
- **ESP-NOW measurements do not reach the S3:** Verify the configured S3 MAC
  address and that both devices are operating on the same Wi-Fi channel.
