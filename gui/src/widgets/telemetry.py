"""Telemetry panel for the values the ESP32-S3 firmware reports.

The panel only displays what it is given. Nothing is shown until the first
sample arrives ("--" rather than 0), and once the link goes stale the last
values stay visible but are greyed out so they are not mistaken for current
readings.

Units follow the firmware: error is plate millimeters, and controller output
and servo commands are microseconds.
"""

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

from telemetry_receiver import LinkState

UNAVAILABLE = "--"
# Full reasons go to the log; the panel only needs enough to spot a problem.
MAX_REASON_CHARS = 90

_STATUS_STYLES = {
    "idle": "background: #5a5a5a; color: white;",
    "live": "background: #2e7d32; color: white;",
    "stale": "background: #b26a00; color: white;",
    "error": "background: #c62828; color: white;",
    "simulated": "background: #6a1b9a; color: white;",
}
_STALE_VALUE_STYLE = "color: gray; font-style: italic;"


class TelemetryWidget(QWidget):
    """Error is target minus measured, as computed by the firmware."""

    def __init__(self, parent=None):
        super().__init__(parent)

        mono = QFont("Consolas")
        mono.setStyleHint(QFont.StyleHint.Monospace)

        self._value_labels = []

        def value_label():
            label = QLabel(UNAVAILABLE)
            label.setFont(mono)
            label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            label.setMinimumWidth(70)
            self._value_labels.append(label)
            return label

        self._status_label = QLabel()
        self._status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._status_label.setWordWrap(True)
        self._status_label.setContentsMargins(6, 4, 6, 4)

        ball_box = QGroupBox("Ball (camera px)")
        ball_grid = QGridLayout(ball_box)
        self._detected_label = value_label()
        self._ball_labels = (value_label(), value_label())
        self._age_label = value_label()
        ball_grid.addWidget(QLabel("Detected"), 0, 0)
        ball_grid.addWidget(self._detected_label, 0, 1)
        ball_grid.addWidget(QLabel("Position X / Y (px)"), 1, 0)
        ball_grid.addWidget(self._ball_labels[0], 1, 1)
        ball_grid.addWidget(self._ball_labels[1], 1, 2)
        ball_grid.addWidget(QLabel("Measurement age (ms)"), 2, 0)
        ball_grid.addWidget(self._age_label, 2, 1)

        controller_box = QGroupBox("Controller")
        controller_grid = QGridLayout(controller_box)
        for column, axis in enumerate(("X", "Y"), start=1):
            header = QLabel(axis)
            header.setAlignment(Qt.AlignmentFlag.AlignRight)
            controller_grid.addWidget(header, 0, column)

        self._controller_labels = {}
        rows = (("error", "Error (mm)"), ("output", "PID output (µs)"), ("servo", "Servo pulse (µs)"))
        for row, (key, title) in enumerate(rows, start=1):
            controller_grid.addWidget(QLabel(title), row, 0)
            labels = (value_label(), value_label())
            controller_grid.addWidget(labels[0], row, 1)
            controller_grid.addWidget(labels[1], row, 2)
            self._controller_labels[key] = labels

        controller_grid.addWidget(QLabel("Firmware servo output"), len(rows) + 1, 0)
        self._enabled_label = value_label()
        controller_grid.addWidget(self._enabled_label, len(rows) + 1, 1, 1, 2)

        link_box = QGroupBox("Link")
        link_grid = QGridLayout(link_box)
        self._sequence_label = value_label()
        self._received_label = value_label()
        self._rejected_label = value_label()
        self._missed_label = value_label()
        for row, (title, label) in enumerate((
            ("Last sequence", self._sequence_label),
            ("Packets received", self._received_label),
            ("Packets rejected", self._rejected_label),
            ("Packets missed (sequence gaps)", self._missed_label),
        )):
            link_grid.addWidget(QLabel(title), row, 0)
            link_grid.addWidget(label, row, 1)
        self._reject_reason_label = QLabel()
        self._reject_reason_label.setWordWrap(True)
        self._reject_reason_label.setStyleSheet("color: gray;")
        link_grid.addWidget(self._reject_reason_label, 4, 0, 1, 2)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._status_label)
        layout.addWidget(ball_box)
        layout.addWidget(controller_box)
        layout.addWidget(link_box)
        layout.addStretch(1)

        self.set_link_state(LinkState.NO_DATA)

    def set_sample(self, sample):
        """Show a TelemetrySample (or anything with the same attributes)."""
        self._detected_label.setText("Yes" if sample.ball_detected else "No")
        _set_pair(self._ball_labels, (sample.ball_x, sample.ball_y), "{:d}")
        self._age_label.setText(f"{sample.measurement_age_ms:d}")
        _set_pair(self._controller_labels["error"], (sample.error_x_mm, sample.error_y_mm), "{:.2f}")
        _set_pair(self._controller_labels["output"], (sample.output_x_us, sample.output_y_us), "{:.2f}")
        _set_pair(self._controller_labels["servo"], (sample.servo_x_us, sample.servo_y_us), "{:d}")
        self._enabled_label.setText("Enabled" if sample.controller_enabled else "Disabled")
        self._sequence_label.setText(str(sample.sequence))

    def set_stats(self, stats):
        """Show ReceiverStats counters."""
        self._received_label.setText(str(stats.received))
        self._rejected_label.setText(str(stats.rejected))
        self._missed_label.setText(str(stats.missed))

    def set_last_rejection(self, reason):
        if reason and len(reason) > MAX_REASON_CHARS:
            reason = reason[: MAX_REASON_CHARS - 3] + "..."
        self._reject_reason_label.setText(f"Last rejected: {reason}" if reason else "")

    def set_link_state(self, state, seconds_since_last=None, timeout=None):
        if state is LinkState.LIVE:
            self._set_status("Live telemetry", "live")
        elif state is LinkState.STALE:
            text = "Telemetry stale"
            if seconds_since_last is not None:
                text += f": no packet for {seconds_since_last:.1f} s"
            if timeout is not None:
                text += f" (timeout {timeout:.1f} s)"
            self._set_status(text, "stale")
        else:
            self._set_status("No telemetry received yet", "idle")
        self._set_values_current(state is LinkState.LIVE)

    def set_error(self, message):
        self._set_status(message, "error")
        self._set_values_current(False)

    def set_simulated(self):
        """Mark everything as mock data, for demos without hardware."""
        self._set_status("Simulated data (not from hardware)", "simulated")
        self._set_values_current(True)

    def _set_status(self, text, kind):
        self._status_label.setText(text)
        self._status_label.setStyleSheet(_STATUS_STYLES[kind] + " font-weight: bold;")

    def _set_values_current(self, current):
        style = "" if current else _STALE_VALUE_STYLE
        for label in self._value_labels:
            label.setStyleSheet(style)


def _set_pair(labels, pair, fmt):
    for label, value in zip(labels, pair):
        label.setText(UNAVAILABLE if value is None else fmt.format(value))


if __name__ == "__main__":
    # Standalone demo with mock data. From gui/: python -m widgets.telemetry
    import sys
    import time

    from PyQt6.QtCore import QTimer
    from PyQt6.QtWidgets import QApplication

    from mock_data import MockCamera, MockTelemetry

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    camera = MockCamera()
    mock = MockTelemetry()
    telemetry = TelemetryWidget()
    telemetry.setWindowTitle("Telemetry (mock data)")
    telemetry.resize(360, 480)
    telemetry.set_simulated()
    start = time.perf_counter()

    def tick():
        t = time.perf_counter() - start
        telemetry.set_sample(mock.sample(camera.ball_position(t)))

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(100)

    telemetry.show()
    sys.exit(app.exec())
