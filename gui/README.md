# Ball and Plate Control System — GUI

The desktop graphical user interface for the **2-DOF Ball and Plate Balancing Platform**, developed using Python and PyQt6.

The GUI provides a workspace for monitoring the ball's position, configuring target setpoints, tuning PID parameters, and observing system telemetry. It is designed to integrate with the ESP32 firmware and computer vision subsystem in later development phases.

## Current Development Status

**Phase:** UI Development

The current focus is building the interface and validating its layout and interactions before integrating hardware.

* [x] Initial project structure
* [ ] Main window and layout
* [x] Camera preview placeholder
* [x] Ball position and target displays
* [ ] X/Y setpoint controls
* [ ] Dual-axis PID tuning panel
* [ ] System control buttons
* [x] Telemetry and plotting placeholders
* [ ] UI interaction testing
* [ ] ESP32 and camera integration

See [`../docs/development/gui-checklist.md`](../docs/development/gui-checklist.md) for the detailed development checklist.

## Technology Stack

| Technology | Purpose                                    |
| ---------- | ------------------------------------------ |
| Python     | Application programming language           |
| PyQt6      | Desktop GUI framework                      |
| OpenCV     | Future camera capture and image processing |
| PySerial   | Future serial communication with the ESP32 |
| PyQtGraph  | Future real-time telemetry plotting        |

## Project Structure

```text
gui/
├── src/
│   ├── __init__.py
│   ├── main.py
│   ├── main_window.py
│   ├── live_telemetry.py
│   ├── mock_data.py
│   ├── monitoring_demo.py
│   ├── telemetry_receiver.py
│   ├── telemetry_simulator.py
│   └── widgets/
│       ├── __init__.py
│       ├── camera_preview.py
│       ├── telemetry.py
│       └── plots.py
├── tests/
│   └── test_telemetry_receiver.py
├── requirements.txt
└── README.md
```

### Module Responsibilities

* `main.py` — Application entry point. Creates the `QApplication`, initializes the main window, and starts the event loop.
* `main_window.py` — Main window, layouts, panels, controls, and UI state.
* `widgets/camera_preview.py` — Camera feed with ball and target markers in image pixels.
* `widgets/telemetry.py` — Ball position, measurement age, error, PID output, servo pulses, firmware output state, and link status, in the firmware's units.
* `telemetry_receiver.py` — UDP telemetry receiver and packet parser. Has no Qt dependency, so it can be tested on its own.
* `live_telemetry.py` — Shows live telemetry from the ESP32-S3 in the telemetry panel.
* `telemetry_simulator.py` — Sends fake telemetry packets in the firmware's format, for trying the live view without hardware.
* `widgets/plots.py` — Scrolling plots of position versus setpoint and X/Y tracking error.
* `mock_data.py` — Mock camera frames and firmware-shaped telemetry for demos without hardware.
* `monitoring_demo.py` — Runs the camera preview, telemetry, and plots together on mock data.
* `tests/` — Automated tests for interface behavior and input validation.
* `requirements.txt` — Python package dependencies.

## Prerequisites

* Windows 10 or later
* Python 3.11–3.14, subject to package compatibility
* Visual Studio Code or another Python-compatible editor
* Git

## Setup

Run the following commands in PowerShell from the `gui/` directory.

### 1. Create a virtual environment

```powershell
python3 -m venv .venv
```

### 2. Activate the environment

```powershell
.\.venv\Scripts\Activate.ps1
```

### 3. Install dependencies

```powershell
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### 4. Run the application

```powershell
$env:PYTHONPATH = "src"
python -m main
```

Run these commands from the `gui/` directory.

### 5. Preview the monitoring panels (optional)

The camera preview, telemetry, and plot panels can be run on mock data without the main window:

```powershell
$env:PYTHONPATH = "src"
python -m monitoring_demo          # all panels together
python -m widgets.camera_preview   # or one panel at a time
python -m widgets.telemetry
python -m widgets.plots
```

### 6. Run the tests

```powershell
python -m unittest discover -s tests -v
```

The tests use only the standard library plus PyQt6, and need no hardware or network connection.

## Live Telemetry

The ESP32-S3 broadcasts telemetry over its own Wi-Fi network. The GUI only listens; it does not send anything to the firmware.

| Setting | Value |
| ------- | ----- |
| Wi-Fi network | `BallPlateSystem` (password `BallPlateSystem1234`, see `firmware/esp32_s3/main/wifi_ap.c`) |
| ESP32-S3 address | `192.168.4.1` |
| Telemetry | UDP broadcast to `192.168.4.255`, port `9002`, one JSON object per datagram |

The firmware sends at most 20 packets per second, and **only while it has a fresh ball detection**. When the ball is removed or the camera stops, packets stop instead of reporting "no ball", so the GUI marks telemetry stale after 1 s without a packet (change this with `--stale-timeout`).

Ball position and error are in camera pixels (160×120 frame). PID output and servo pulses are in microseconds. "Firmware servo output" shows the firmware's `controller_enabled` flag, which is currently always *Disabled* because `CONTROL_OUTPUT_ENABLED` is 0 in `pid_controller.c`. Pitch, roll, PID gains, the target position, and elapsed time are not in the telemetry packet, so they are not shown.

### Trying it without hardware

In two terminals, both in `gui/` with `$env:PYTHONPATH = "src"`:

```powershell
python -m live_telemetry
python -m telemetry_simulator --drop 0.1 --pause-every 5
```

`--drop` skips some packets so they appear as sequence gaps, and `--pause-every` pauses the simulator now and then so the panel goes stale.

### Hardware smoke test

1. Power the ESP32-S3 and the ESP32-CAM.
2. Connect this computer to the `BallPlateSystem` Wi-Fi network. Windows will report "No internet", which is expected.
3. From `gui/`, run:

   ```powershell
   $env:PYTHONPATH = "src"
   python -m live_telemetry
   ```

   If Windows Firewall asks whether Python may communicate on this network, allow it. Otherwise broadcast packets are silently blocked.
4. The panel shows **No telemetry received yet** until a packet arrives.
5. Place the ball on the plate. The banner turns green (**Live telemetry**), the position follows the ball, and *Packets received* increases.
6. Remove the ball. Within about 1 s the banner turns orange (**Telemetry stale**) and the values grey out.
7. Close the window. The terminal returns to the prompt, which means the receiver stopped and released the port.

**Troubleshooting**

* *Cannot listen on UDP port 9002:* another program, such as a second copy of the GUI, is already using the port.
* *Stays on "No telemetry received yet":* check that you are on `BallPlateSystem` (`ipconfig` should show a `192.168.4.x` address), check the firewall prompt, and check the ESP32-S3 serial log for `UDP telemetry initialized`.
* *Packets rejected keeps rising:* the reason is shown under the counters, and full details are printed to the terminal.

## Interface Components

### Main Window

Provides the overall application layout and system status.

### Camera and Ball Tracking

Reserves space for the future camera feed, detected ball position, and target position.

### Setpoint Controls

Allows the user to configure the desired X/Y ball coordinates in millimeters.

### PID Tuning

Provides separate proportional, integral, and derivative parameter controls for the X and Y axes.

### System Controls

Contains Start, Stop, and Reset controls. During UI development, these controls only update the interface state; they do not operate the physical platform.

### Telemetry and Plots

Shows live telemetry from the ESP32-S3 (see [Live Telemetry](#live-telemetry)). The plots still use mock data.


## Development Roadmap

1. Complete the main window and layout.
2. Implement the camera and tracking placeholders.
3. Implement setpoint and PID input controls.
4. Add UI-only Start, Stop, and Reset state transitions.
5. Add telemetry and plotting placeholders.
6. Test layout behavior and input validation.
7. Integrate the camera and computer vision pipeline.
8. Integrate serial communication with the ESP32 firmware.
