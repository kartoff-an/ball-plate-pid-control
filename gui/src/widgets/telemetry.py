"""Telemetry panel: measured and target positions, tracking error, servo angles.

Like the camera preview, the panel only displays values it is given. Pass
None for anything that is unavailable and it is shown as "--" rather than 0.
"""

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

UNAVAILABLE = "--"


class TelemetryWidget(QWidget):
    """Positions are plate coordinates in millimeters; angles are in degrees.

    Error is reported as target minus measured.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._measured = None
        self._target = None

        mono = QFont("Consolas")
        mono.setStyleHint(QFont.StyleHint.Monospace)

        def value_label():
            label = QLabel(UNAVAILABLE)
            label.setFont(mono)
            label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            label.setMinimumWidth(70)
            return label

        position_box = QGroupBox("Ball Position (mm)")
        position_grid = QGridLayout(position_box)
        for column, axis in enumerate(("X", "Y"), start=1):
            header = QLabel(axis)
            header.setAlignment(Qt.AlignmentFlag.AlignRight)
            position_grid.addWidget(header, 0, column)

        self._position_labels = {}
        for row, name in enumerate(("Measured", "Target", "Error"), start=1):
            position_grid.addWidget(QLabel(name), row, 0)
            labels = (value_label(), value_label())
            position_grid.addWidget(labels[0], row, 1)
            position_grid.addWidget(labels[1], row, 2)
            self._position_labels[name] = labels

        servo_box = QGroupBox("Plate")
        servo_grid = QGridLayout(servo_box)
        self._servo_labels = (value_label(), value_label())
        self._elapsed_label = value_label()
        servo_grid.addWidget(QLabel("Servo X angle (°)"), 0, 0)
        servo_grid.addWidget(self._servo_labels[0], 0, 1)
        servo_grid.addWidget(QLabel("Servo Y angle (°)"), 1, 0)
        servo_grid.addWidget(self._servo_labels[1], 1, 1)
        servo_grid.addWidget(QLabel("Elapsed time (s)"), 2, 0)
        servo_grid.addWidget(self._elapsed_label, 2, 1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(position_box)
        layout.addWidget(servo_box)
        layout.addStretch(1)

    def set_measured_position(self, position):
        self._measured = position
        self._refresh_positions()

    def set_target_position(self, position):
        self._target = position
        self._refresh_positions()

    def set_servo_angles(self, angles):
        _set_pair(self._servo_labels, angles)

    def set_elapsed_time(self, seconds):
        self._elapsed_label.setText(_format(seconds))

    def _refresh_positions(self):
        error = None
        if self._measured is not None and self._target is not None:
            error = tuple(t - m for m, t in zip(self._measured, self._target))

        _set_pair(self._position_labels["Measured"], self._measured)
        _set_pair(self._position_labels["Target"], self._target)
        _set_pair(self._position_labels["Error"], error)


def _format(value):
    return UNAVAILABLE if value is None else f"{value:.1f}"


def _set_pair(labels, pair):
    values = pair if pair is not None else (None, None)
    for label, value in zip(labels, values):
        label.setText(_format(value))


if __name__ == "__main__":
    # Standalone demo with mock data. From gui/: python -m widgets.telemetry
    import sys
    import time

    from PyQt6.QtCore import QTimer
    from PyQt6.QtWidgets import QApplication

    from mock_data import MockCamera, mock_servo_angles, pixels_to_mm

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    camera = MockCamera()
    telemetry = TelemetryWidget()
    telemetry.setWindowTitle("Telemetry (mock data)")
    telemetry.resize(320, 300)

    target = (0.0, 0.0)
    telemetry.set_target_position(target)
    start = time.perf_counter()

    def tick():
        t = time.perf_counter() - start
        measured = pixels_to_mm(camera.ball_position(t))
        telemetry.set_measured_position(measured)
        telemetry.set_servo_angles(mock_servo_angles(measured, target))
        telemetry.set_elapsed_time(t)

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(100)

    telemetry.show()
    sys.exit(app.exec())
