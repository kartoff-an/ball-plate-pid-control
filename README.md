# Ball and Plate PID Control

A 2-DOF ball-and-plate balancing platform developed as a Feedback and Control Systems project. The system uses computer vision to track a ball and dual-axis PID control to maintain its position on a tilting plate.

## System Architecture

* **Firmware:** ESP32-CAM and ESP32-S3 with ESP-IDF.
* **GUI:** Python and PyQt6 for system monitoring and control.
* **Computer Vision:** OpenCV for ball tracking and position estimation.
* **Actuation:** Two servo motors for independent plate-axis control.

## Repository Structure

```text
ball-plate-pid-control/
├── firmware/
│   ├── esp32_cam/  # ESP32-CAM firmware
│   └── esp32_s3/   # ESP32-S3 firmware
├── gui/         # Desktop control interface
├── docs/        # System documentation
└── README.md
```

## Project Status

**In development.** Software modules, hardware integration, and control algorithms are being implemented incrementally. Camera and communication features may remain placeholders during initial GUI development.

## Objectives

* Track ball position using computer vision.
* Implement independent PID controllers for two axes.
* Stabilize the ball within a defined target region.
* Monitor system state and control parameters through the desktop GUI.
