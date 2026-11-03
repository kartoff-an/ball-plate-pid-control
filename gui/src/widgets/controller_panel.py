import logging

import config

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QDoubleSpinBox,
    QSpinBox,
)

from communication.esp_sender import ESPSender
import state

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
        target_layout.addWidget(QLabel("Y"), 0, 2)
        target_layout.addWidget(self.target_y, 0, 3)

        self.target_button = QPushButton("Set Target")
        self.target_button.clicked.connect(self._set_target)
        target_layout.addWidget(self.target_button, 1, 0, 1, 4)

        # Servo neutral calibration
        servo_box = QGroupBox("Servo Neutral Calibration")
        servo_layout = QGridLayout(servo_box)

        self.servo_neutral_x = self._integer_input(1000, 2000, 1450, 5)
        self.servo_neutral_y = self._integer_input(1000, 2000, 1500, 5)

        servo_layout.addWidget(QLabel("X (CH1)"), 0, 0)
        servo_layout.addWidget(self.servo_neutral_x, 0, 1)
        servo_layout.addWidget(QLabel("Y (CH0)"), 0, 2)
        servo_layout.addWidget(self.servo_neutral_y, 0, 3)

        self.apply_neutral_button = QPushButton("Apply Neutral")
        self.apply_neutral_button.clicked.connect(self._apply_servo_neutral)
        servo_layout.addWidget(self.apply_neutral_button, 1, 0, 1, 4)

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

        gain_defaults = {
            "kp": 0.5,
            "ki": 0.0,
            "kd": 0.0,
        }

        for row, gain_name in enumerate(("kp", "ki", "kd"), start=1):
            pid_layout.addWidget(QLabel(gain_name.upper()), row, 0)

            for axis, column in (("x", 1), ("y", 2)):
                maximum, step = gain_limits[gain_name]
                spin = self._number_input(0.0, maximum, gain_defaults[gain_name], step, 3)
                self.gains[f"{gain_name}_{axis}"] = spin
                pid_layout.addWidget(spin, row, column)

        self.apply_gains_button = QPushButton("Apply PID Gains")
        self.apply_gains_button.clicked.connect(self._apply_gains)
        pid_layout.addWidget(self.apply_gains_button, 4, 0, 1, 3)

        # Kalman filter calibration
        kalman_box = QGroupBox("Kalman Filter")
        kalman_layout = QGridLayout(kalman_box)

        self.kalman_measurement_variance = self._number_input(0.01, 1000.0, 4.0, 0.1, 3)
        self.kalman_acceleration_variance = self._number_input(0.01, 100000.0, 500.0, 5.0, 3)

        kalman_layout.addWidget(QLabel("Measurement variance"), 0, 0)
        kalman_layout.addWidget(self.kalman_measurement_variance, 0, 1)
        kalman_layout.addWidget(QLabel("Acceleration variance"), 1, 0)
        kalman_layout.addWidget(self.kalman_acceleration_variance, 1, 1)

        self.apply_kalman_button = QPushButton("Apply Kalman")
        self.apply_kalman_button.clicked.connect(self._apply_kalman)
        kalman_layout.addWidget(self.apply_kalman_button, 2, 0, 1, 2)

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

        calibration_layout = QHBoxLayout()
        calibration_layout.addWidget(target_box)
        calibration_layout.addWidget(servo_box)

        layout.addLayout(calibration_layout)
        layout.addWidget(pid_box)
        layout.addWidget(kalman_box)
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

    @staticmethod
    def _integer_input(minimum, maximum, default_value, step):
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(default_value)
        spin.setSingleStep(step)
        spin.setSuffix(" us")
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

        def update_target():
            self.sender.set_target(x, y)
            with state.app_state._lock:
                state.app_state.target.x = x
                state.app_state.target.y = y

        self._run_command(f"target X={x:.1f}, Y={y:.1f} mm", update_target)

    def _apply_servo_neutral(self):
        x_us = self.servo_neutral_x.value()
        y_us = self.servo_neutral_y.value()

        self._run_command(
            f"servo neutral X={x_us}, Y={y_us} us",
            lambda: self.sender.set_servo_neutral(x_us, y_us),
        )

    def _apply_gains(self):
        kp_x = self.gains["kp_x"].value()
        ki_x = self.gains["ki_x"].value()
        kd_x = self.gains["kd_x"].value()

        kp_y = self.gains["kp_y"].value()
        ki_y = self.gains["ki_y"].value()
        kd_y = self.gains["kd_y"].value()

        with state.app_state._lock:
            state.app_state.pid.kp_x = kp_x
            state.app_state.pid.ki_x = ki_x
            state.app_state.pid.kd_x = kd_x
            state.app_state.pid.kp_y = kp_y
            state.app_state.pid.ki_y = ki_y
            state.app_state.pid.kd_y = kd_y

        self._run_command("PID gains", lambda: self.sender.set_pid_gains(
                kp_x, ki_x, kd_x,
                kp_y, ki_y, kd_y,
            ),
        )

    def _apply_kalman(self):
        measurement_variance = self.kalman_measurement_variance.value()
        acceleration_variance = self.kalman_acceleration_variance.value()

        self._run_command(
            f"Kalman variance Q={measurement_variance:.3f}, R={acceleration_variance:.3f}",
            lambda: self.sender.set_kalman_filter(measurement_variance, acceleration_variance),
        )

    def _start(self):
        self._run_command("controller start", self.sender.start)

    def _stop(self):
        self._run_command("controller stop", self.sender.stop)

    def _reset_pid(self):
        self._run_command("PID reset", self.sender.reset_pid)