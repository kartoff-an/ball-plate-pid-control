
import logging

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QDoubleSpinBox,
)

from communication.esp_sender import ESPSender

log = logging.getLogger(__name__)


class ControllerPanel(QWidget):
    def __init__(self, sender: ESPSender, parent=None):
        super().__init__(parent)

        self.sender = sender

        self.status_label = QLabel("Controller commands ready")
        self.status_label.setWordWrap(True)

        # Target position
        target_box = QGroupBox("Target Position (mm)")
        target_layout = QGridLayout(target_box)

        self.target_x = self._number_input(-110.0, 110.0, 0.0, 1.0, 1)
        self.target_y = self._number_input(-110.0, 110.0, 0.0, 1.0, 1)

        target_layout.addWidget(QLabel("X"), 0, 0)
        target_layout.addWidget(self.target_x, 0, 1)
        target_layout.addWidget(QLabel("Y"), 1, 0)
        target_layout.addWidget(self.target_y, 1, 1)

        self.target_button = QPushButton("Set Target")
        self.target_button.clicked.connect(self._set_target)
        target_layout.addWidget(self.target_button, 2, 0, 1, 2)

        # PID gains
        pid_box = QGroupBox("PID Gains")
        pid_layout = QGridLayout(pid_box)

        pid_layout.addWidget(QLabel("Gain"), 0, 0)
        pid_layout.addWidget(QLabel("X"), 0, 1)
        pid_layout.addWidget(QLabel("Y"), 0, 2)

        self.gains = {}

        gain_limits = {
            "kp": (10.0, 0.001),
            "ki": (2.0, 0.001),
            "kd": (10.0, 0.001),
        }

        for row, gain_name in enumerate(("kp", "ki", "kd"), start=1):
            pid_layout.addWidget(QLabel(gain_name.upper()), row, 0)

            for axis, column in (("x", 1), ("y", 2)):
                maximum, step = gain_limits[gain_name]
                spin = self._number_input(0.0, maximum, 0.0, step, 3)
                self.gains[f"{gain_name}_{axis}"] = spin
                pid_layout.addWidget(spin, row, column)

        self.apply_gains_button = QPushButton("Apply PID Gains")
        self.apply_gains_button.clicked.connect(self._apply_gains)
        pid_layout.addWidget(self.apply_gains_button, 4, 0, 1, 3)

        # Controller actions
        actions_box = QGroupBox("Controller")
        actions_layout = QHBoxLayout(actions_box)

        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.reset_button = QPushButton("Reset PID")

        self.start_button.clicked.connect(self._start)
        self.stop_button.clicked.connect(self._stop)
        self.reset_button.clicked.connect(self._reset_pid)

        actions_layout.addWidget(self.start_button)
        actions_layout.addWidget(self.stop_button)
        actions_layout.addWidget(self.reset_button)

        layout = QVBoxLayout(self)
        layout.addWidget(target_box)
        layout.addWidget(pid_box)
        layout.addWidget(actions_box)
        layout.addWidget(self.status_label)

    @staticmethod
    def _number_input(minimum, maximum, default_value, step, decimals):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(default_value)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        return spin

    def _run_command(self, description, callback):
        try:
            callback()
            self.status_label.setText(f"Sent: {description}")
            log.info("ESP32 command sent: %s", description)
        except (OSError, ValueError) as error:
            self.status_label.setText(f"Command failed: {error}")
            log.exception("Could not send ESP32 command")

    def _set_target(self):
        x = self.target_x.value()
        y = self.target_y.value()

        self._run_command(f"target X={x:.1f}, Y={y:.1f} mm", lambda: self.sender.set_target(x, y))

    def _apply_gains(self):
        kp_x = self.gains["kp_x"].value()
        ki_x = self.gains["ki_x"].value()
        kd_x = self.gains["kd_x"].value()

        kp_y = self.gains["kp_y"].value()
        ki_y = self.gains["ki_y"].value()
        kd_y = self.gains["kd_y"].value()

        self._run_command("PID gains", lambda: self.sender.set_pid_gains(
                kp_x, ki_x, kd_x,
                kp_y, ki_y, kd_y,
            ),
        )

    def _start(self):
        self._run_command("controller start", self.sender.start)

    def _stop(self):
        self._run_command("controller stop", self.sender.stop)

    def _reset_pid(self):
        self._run_command("PID reset", self.sender.reset_pid)
