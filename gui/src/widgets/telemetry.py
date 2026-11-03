
"""Telemetry panel for the values the ESP32-S3 firmware reports.

The panel only displays what it is given. Nothing is shown until the first
sample arrives ("--" rather than 0), and once the link goes stale the last
values stay visible but are greyed out so they are not mistaken for current
readings.

Units follow the firmware: position and error are plate millimeters,
velocity is millimeters per second, and controller output and servo commands
are microseconds.
"""

from PyQt6.QtCore import Qt, pyqtSignal, QSignalBlocker
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QComboBox, QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

from communication.telemetry_receiver import LinkState

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

TELEMETRY_MODES = {
    "Position": "position",
    "Velocity": "velocity",
    "Servo": "servo"
}


class TelemetryWidget(QWidget):
    """Error is target minus measured, as computed by the firmware."""

    telemetry_mode_changed = pyqtSignal(str)

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

        mode_box = QGroupBox("Telemetry response")
        mode_layout = QGridLayout(mode_box)
        mode_layout.addWidget(QLabel("Mode"), 0, 0)

        self._mode_combo = QComboBox()
        for label, mode in TELEMETRY_MODES.items():
            self._mode_combo.addItem(label, mode)

        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_layout.addWidget(self._mode_combo, 0, 1)

        ball_box = QGroupBox("Ball (mm)")
        ball_grid = QGridLayout(ball_box)

        self._detected_label = value_label()
        self._ball_labels = (value_label(), value_label())
        self._velocity_labels = (value_label(), value_label())
        self._target_labels = (value_label(), value_label())
        self._age_label = value_label()

        ball_rows = (
            ("detected", "Detected", (self._detected_label,)),
            ("position", "Position X / Y (mm)", self._ball_labels),
            ("velocity", "Velocity X / Y (mm/s)", self._velocity_labels),
            ("target", "Target X / Y (mm)", self._target_labels),
            ("age", "Measurement age (ms)", (self._age_label,)),
        )

        self._ball_rows = {}

        for row, (key, title, labels) in enumerate(ball_rows):
            title_label = QLabel(title)
            ball_grid.addWidget(title_label, row, 0)

            for column, label in enumerate(labels, start=1):
                ball_grid.addWidget(label, row, column)

            self._ball_rows[key] = (title_label, *labels)

        controller_box = QGroupBox("Controller")
        controller_grid = QGridLayout(controller_box)

        for column, axis in enumerate(("X", "Y"), start=1):
            header = QLabel(axis)
            header.setAlignment(Qt.AlignmentFlag.AlignRight)
            controller_grid.addWidget(header, 0, column)

        self._controller_labels = {}
        controller_rows = (
            ("error", "Error (mm)"),
            ("pid_output", "PID output (µs)"),
            ("output", "Controller output (µs)"),
            ("neutral", "Servo neutral (µs)"),
            ("servo", "Servo pulse (µs)"),
            ("saturated", "Servo saturated")
        )

        self._controller_rows = {}

        for row, (key, title) in enumerate(controller_rows, start=1):
            title_label = QLabel(title)
            labels = (value_label(), value_label())

            controller_grid.addWidget(title_label, row, 0)
            controller_grid.addWidget(labels[0], row, 1)
            controller_grid.addWidget(labels[1], row, 2)

            self._controller_labels[key] = labels
            self._controller_rows[key] = (title_label, *labels)

        enabled_row = len(controller_rows) + 1
        self._enabled_title = QLabel("Firmware controller")
        self._enabled_label = value_label()

        controller_grid.addWidget(self._enabled_title, enabled_row, 0)
        controller_grid.addWidget(self._enabled_label, enabled_row, 1, 1, 2)

        self._sequence_label = value_label()
        self._received_label = value_label()
        self._rejected_label = value_label()
        self._missed_label = value_label()
        self._reject_reason_label = QLabel()
        self._reject_reason_label.setWordWrap(True)
        self._reject_reason_label.setStyleSheet("color: gray;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._status_label)
        layout.addWidget(mode_box)
        layout.addWidget(ball_box)
        layout.addWidget(controller_box)
        layout.addStretch(1)

        self._set_mode_visibility("position")
        self.set_link_state(LinkState.NO_DATA)

    def _on_mode_changed(self, index):
        mode = self._mode_combo.itemData(index)
        if mode is None:
            return

        self._set_mode_visibility(mode)
        self.telemetry_mode_changed.emit(mode)

    def set_sample(self, sample):
        """Show a telemetry sample using the fields for its response mode."""

        mode = getattr(sample, "mode", self._mode_combo.currentData())

        if mode not in TELEMETRY_MODES.values():
            return

        self._set_selected_mode(mode)

        detected = getattr(sample, "ball_detected", None)
        self._detected_label.setText(
            UNAVAILABLE if detected is None
            else "Yes" if detected else "No"
        )

        self._set_pair(
            self._ball_labels,
            (getattr(sample, "ball_x_mm", None),
             getattr(sample, "ball_y_mm", None)),
            "{:.2f}",
        )

        self._set_pair(
            self._velocity_labels,
            (getattr(sample, "velocity_x_mm_s", None),
             getattr(sample, "velocity_y_mm_s", None)),
            "{:.2f}",
        )

        self._set_pair(
            self._target_labels,
            (getattr(sample, "target_x_mm", None),
             getattr(sample, "target_y_mm", None)),
            "{:.2f}",
        )

        age = getattr(sample, "measurement_age_ms", None)
        self._age_label.setText(
            UNAVAILABLE if age is None else str(age)
        )

        self._set_pair(
            self._controller_labels["error"],
            (getattr(sample, "error_x_mm", None),
             getattr(sample, "error_y_mm", None)),
            "{:.2f}",
        )

        self._set_pair(
            self._controller_labels["pid_output"],
            (getattr(sample, "pid_output_x_us", None),
             getattr(sample, "pid_output_y_us", None)),
            "{:.2f}",
        )

        self._set_pair(
            self._controller_labels["output"],
            (getattr(sample, "output_x_us", None),
             getattr(sample, "output_y_us", None)),
            "{:.2f}",
        )

        self._set_pair(
            self._controller_labels["neutral"],
            (getattr(sample, "servo_neutral_x_us", None),
             getattr(sample, "servo_neutral_y_us", None)),
            "{:d}",
        )

        self._set_pair(
            self._controller_labels["servo"],
            (getattr(sample, "servo_x_us", None),
             getattr(sample, "servo_y_us", None)),
            "{:d}",
        )

        self._set_pair(
            self._controller_labels["saturated"],
            (getattr(sample, "servo_saturated_x", None),
             getattr(sample, "servo_saturated_y", None)),
            "{}",
        )

        enabled = getattr(sample, "controller_enabled", None)
        self._enabled_label.setText(
            UNAVAILABLE if enabled is None
            else "Enabled" if enabled else "Disabled"
        )

        self._sequence_label.setText(
            str(getattr(sample, "sequence", UNAVAILABLE))
        )

    def _set_selected_mode(self, mode):
        index = self._mode_combo.findData(mode)

        if index < 0:
            return

        blocker = QSignalBlocker(self._mode_combo)
        self._mode_combo.setCurrentIndex(index)
        del blocker

        self._set_mode_visibility(mode)

    def _set_mode_visibility(self, mode):
        visibility = {
            "position": {
                "detected", "position", "target", "age", "error"
            },
            "velocity": {
                "detected", "position", "velocity", "target", "age"
            },
            "servo": {
                "error", "pid_output", "output", "neutral", "servo", "saturated"
            },
        }

        visible = visibility.get(mode, set())

        for key, widgets in self._ball_rows.items():
            for widget in widgets:
                widget.setVisible(key in visible)

        for key, widgets in self._controller_rows.items():
            for widget in widgets:
                widget.setVisible(key in visible)

        # Controller enabled is common to all telemetry modes.
        self._enabled_title.setVisible(True)
        self._enabled_label.setVisible(True)

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

    @staticmethod
    def _set_pair(labels, pair, fmt):
        for label, value in zip(labels, pair):
            if value is None:
                label.setText(UNAVAILABLE)
            elif fmt == "{}":
                label.setText("Yes" if value else "No")
            else:
                label.setText(fmt.format(value))
