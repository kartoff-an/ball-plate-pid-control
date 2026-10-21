"""Shows the camera preview, telemetry, and plots together, all fed by mock data.

This is only a preview of the issue #3 panels. The real dashboard layout
belongs to the main window (issue #2).

From gui/: python -m monitoring_demo
"""

import sys
import time

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication, QHBoxLayout, QVBoxLayout, QWidget

from mock_data import FRAME_HEIGHT, FRAME_WIDTH, MockCamera, mock_servo_angles, pixels_to_mm
from widgets.camera_preview import CameraPreviewWidget
from widgets.plots import PlotsWidget
from widgets.telemetry import TelemetryWidget

TARGET_MM = (0.0, 0.0)
TARGET_PX = (FRAME_WIDTH / 2, FRAME_HEIGHT / 2)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    camera = MockCamera()
    preview = CameraPreviewWidget(FRAME_WIDTH, FRAME_HEIGHT)
    telemetry = TelemetryWidget()
    plots = PlotsWidget()

    preview.set_status("Mock camera: 160x120 grayscale")
    preview.set_target_position(TARGET_PX)
    telemetry.set_target_position(TARGET_MM)

    left = QVBoxLayout()
    left.addWidget(preview, stretch=1)
    left.addWidget(telemetry)

    window = QWidget()
    window.setWindowTitle("Monitoring Panels (mock data)")
    layout = QHBoxLayout(window)
    layout.addLayout(left, stretch=2)
    layout.addWidget(plots, stretch=3)
    window.resize(1200, 750)

    start = time.perf_counter()

    def tick():
        t = time.perf_counter() - start
        ball_px = camera.ball_position(t)
        measured = pixels_to_mm(ball_px)

        preview.set_frame(camera.frame(t))
        preview.set_ball_position(ball_px)
        telemetry.set_measured_position(measured)
        telemetry.set_servo_angles(mock_servo_angles(measured, TARGET_MM))
        telemetry.set_elapsed_time(t)
        plots.add_sample(t, measured, TARGET_MM)

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(33)

    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
