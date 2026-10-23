# ESP32-S3 Controller Firmware

This ESP-IDF project is the controller side of the ball-and-plate system. It
contains modules for creating a Wi-Fi access point, receiving ball measurements
from the ESP32-CAM over ESP-NOW, driving a PCA9685 PWM controller, and running
two-axis PID control.

## Hardware

- ESP32-S3 development board.
- PCA9685 PWM driver connected to I2C at address `0x40`.
- Servos connected to PCA9685 channels 0 and 1 for the two plate axes.
- ESP32-CAM running the companion firmware in `../esp32_cam`.

Use a suitable external supply for the servos and connect its ground to the
controller/PCA9685 ground. Do not assume the ESP32-S3 board can supply servo
power.

## Configure the connection

The access point configuration is in `main/wifi_ap.c`. Its SSID and password
must match the station credentials in
`../esp32_cam/main/wifi_station.c`. The current AP channel is 1; the camera and
controller must use a compatible channel for ESP-NOW.

The ESP32-CAM sends measurements to the S3 using the packed
`ball_measurement_t` payload defined in `main/include/ball_measurement.h` and
the camera project's matching header. Keep the two definitions and protocol
version in sync if the payload changes.

## Build, flash, and monitor

Open an ESP-IDF-enabled terminal at the repository root, then run:

```sh
cd firmware/esp32_s3
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial port for the ESP32-S3 (for example, `COM5` on
Windows or `/dev/ttyUSB0` on Linux). Exit the serial monitor with `Ctrl+]`.

## Source layout

- `main/app_main.c` — Current application startup and PCA9685 servo sweep.
- `main/wifi_ap.c` — Wi-Fi access-point setup.
- `main/ball_receiver.c` — ESP-NOW measurement receiver and latest-measurement
  storage.
- `main/pid_controller.c` — Prototype two-axis PID task and servo output logic.
- `main/pca9685.c` — I2C initialization and PCA9685 pulse generation.
- `main/include/` — Public module headers and the shared measurement payload.

## Troubleshooting

- **Build reports the wrong target:** Run `idf.py set-target esp32s3` from this
  project directory, then rebuild.
- **PCA9685 initialization fails:** Check SDA/SCL wiring, I2C address, power,
  and common ground.
- **No Wi-Fi access point appears:** The current `app_main` does not call
  `wifi_ap_init`; enabling the AP is part of the ongoing integration.
- **No measurements arrive:** The current `app_main` does not call
  `ball_receiver_init`. Once enabled, check the camera's peer MAC, matching
  measurement payload definitions, and Wi-Fi channel.
