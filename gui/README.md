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
│   ├── mock_data.py
│   ├── monitoring_demo.py
│   └── widgets/
│       ├── __init__.py
│       ├── camera_preview.py
│       ├── telemetry.py
│       └── plots.py
├── tests/
├── requirements.txt
└── README.md
```

### Module Responsibilities

* `main.py` — Application entry point. Creates the `QApplication`, initializes the main window, and starts the event loop.
* `main_window.py` — Main window, layouts, panels, controls, and UI state.
* `widgets/camera_preview.py` — Camera feed with ball and target markers in image pixels.
* `widgets/telemetry.py` — Measured and target position, tracking error, servo angles, and elapsed time.
* `widgets/plots.py` — Scrolling plots of position versus setpoint and X/Y tracking error.
* `mock_data.py` — Mock camera frames and telemetry used until the hardware is integrated.
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

Reserves space for plate pitch, plate roll, position error, elapsed time, and future real-time graphs.


## Development Roadmap

1. Complete the main window and layout.
2. Implement the camera and tracking placeholders.
3. Implement setpoint and PID input controls.
4. Add UI-only Start, Stop, and Reset state transitions.
5. Add telemetry and plotting placeholders.
6. Test layout behavior and input validation.
7. Integrate the camera and computer vision pipeline.
8. Integrate serial communication with the ESP32 firmware.
