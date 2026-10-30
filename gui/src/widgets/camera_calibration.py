import json
import logging
import socket
import struct
import time
from pathlib import Path

import numpy as np

from PyQt6.QtCore import Qt, QTimer, QRectF
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QGroupBox,
    QFormLayout,
    QSpinBox,
    QDoubleSpinBox,
    QFileDialog,
    QMessageBox,
)

import config
from communication.camera_receiver import FrameFetcher
from state import app_state

log = logging.getLogger(__name__)


TUNING_PACKET = struct.Struct("!4sIBHHfffff")
ACK_PACKET = struct.Struct("!4sIB")

CROP_LEFT = 20
CROP_TOP = 0
CROP_WIDTH = 120
CROP_HEIGHT = 120

CROP_RIGHT = CROP_LEFT + CROP_WIDTH
CROP_BOTTOM = CROP_TOP + CROP_HEIGHT

TARGET_RADIUS_PX = 8
BALL_MARKER_RADIUS_PX = 3

TELEMETRY_TIMEOUT_S = 0.5

TARGET_X = CROP_LEFT + CROP_WIDTH / 2
TARGET_Y = CROP_TOP + CROP_HEIGHT / 2

DEFAULTS = {
    "threshold": 100,
    "min_area": 8,
    "max_area": 1000,
    "min_aspect": 0.75,
    "max_aspect": 1.33,
    "min_fill": 0.45,
    "max_jump": 15.0,
    "alpha": 0.40,
}


# Camera Preview
class CameraPreviewLabel(QLabel):

    def __init__(self, parent=None):
        super().__init__("Waiting for camera...", parent)

        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(400, 300)
        self.setStyleSheet("background-color: #111; color: white;")

    def set_ball_telemetry(self, sample):
        if sample is None:
            with app_state._lock:
                app_state.ball.detected = False
                app_state.ball.x = None
                app_state.ball.y = None
                app_state.ball.received_at = None
                app_state.ball.measurement_age_ms = None
        else:
            with app_state._lock:
                app_state.ball.detected = sample.ball_detected
                app_state.ball.x = sample.ball_x if sample.ball_detected else None
                app_state.ball.y = sample.ball_y if sample.ball_detected else None
                app_state.ball.received_at = sample.received_at

        self.update()

    def clear_ball_telemetry(self):
        with app_state._lock:
            app_state.ball.detected = False
            app_state.ball.x = None
            app_state.ball.y = None
            app_state.ball.received_at = None
            app_state.ball.measurement_age_ms = None

    def paintEvent(self, event):
        super().paintEvent(event)

        frame_width = config.CAMERA_FRAME_WIDTH
        frame_height = config.CAMERA_FRAME_HEIGHT

        scale = min(self.width() / frame_width, self.height() / frame_height)

        displayed_width = frame_width * scale
        displayed_height = frame_height * scale

        offset_x = (self.width() - displayed_width) / 2
        offset_y = (self.height() - displayed_height) / 2

        def map_x(x):
            return offset_x + x * scale

        def map_y(y):
            return offset_y + y * scale

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Detection crop
        crop_rect = QRectF(
            map_x(CROP_LEFT),
            map_y(CROP_TOP),
            CROP_WIDTH * scale,
            CROP_HEIGHT * scale,
        )

        painter.setPen(QPen(QColor("#22C55E"), 2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRect(crop_rect)

        # Target position
        radius = TARGET_RADIUS_PX * scale

        target_x = map_x(TARGET_X)
        target_y = map_y(TARGET_Y)

        painter.setPen(QPen(QColor("#22C55E"), 2))
        painter.drawEllipse(
            int(target_x - radius),
            int(target_y - radius),
            max(1, int(2 * radius)),
            max(1, int(2 * radius)),
        )

        # Ball telemetry marker
        with app_state._lock:
            ball_detected = app_state.ball.detected
            ball_x = app_state.ball.x
            ball_y = app_state.ball.y
            received_at = app_state.ball.received_at

        telemetry_is_fresh = (
            received_at is not None
            and time.monotonic() - received_at <= TELEMETRY_TIMEOUT_S
        )

        if (
            ball_detected
            and telemetry_is_fresh
            and ball_x is not None
            and ball_y is not None
            and 0 <= ball_x < frame_width
            and 0 <= ball_y < frame_height
        ):
            display_x = map_x(ball_x)
            display_y = map_y(ball_y)

            radius = BALL_MARKER_RADIUS_PX * scale

            painter.setPen(QPen(QColor("#EF4444"), 2))
            painter.setBrush(QColor("#EF4444"))

            painter.drawEllipse(
                int(display_x - radius),
                int(display_y - radius),
                max(1, int(2 * radius)),
                max(1, int(2 * radius)),
            )

        painter.end()


# Camera Calibration and Vision Tuning

class CameraCalibrationPage(QWidget):

    def __init__(self, on_stop):
        super().__init__()

        self.on_stop = on_stop

        self.receiver = None
        self.last_sequence = -1

        self.settings = DEFAULTS.copy()

        self.sequence = 0
        self.last_ack_seq = None
        self.last_ack_time = 0.0
        self.last_send_time = 0.0

        # Tuning UDP socket
        self.tune_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tune_sock.bind(("0.0.0.0", 0))
        self.tune_sock.setblocking(False)

        # Preview widgets
        self.image_label = CameraPreviewLabel()

        self.threshold_label = QLabel("Waiting for camera...")
        self.threshold_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.threshold_label.setMinimumSize(220, 220)
        self.threshold_label.setStyleSheet("background-color: #111; color: white;")

        self.frame_status = QLabel("Camera stopped")
        self.telemetry_status = QLabel("Telemetry unavailable")

        self.tuning_status = QLabel("No tuning acknowledgment received")

        self.stats_label = QLabel("White pixels: —")

        # Parameter controls
        controls = QGroupBox("Runtime detector parameters")

        form = QFormLayout(controls)

        self.threshold = self._spin(0, 255, DEFAULTS["threshold"])
        self.min_area = self._spin(1, 14400, DEFAULTS["min_area"])
        self.max_area = self._spin(1, 14400, DEFAULTS["max_area"])
        self.min_aspect = self._double(0.10, 3.00, DEFAULTS["min_aspect"])
        self.max_aspect = self._double(0.10, 5.00, DEFAULTS["max_aspect"])
        self.min_fill = self._double(0.05, 1.00, DEFAULTS["min_fill"])
        self.max_jump = self._double(1.0, 120.0, DEFAULTS["max_jump"], 1.0)
        self.alpha = self._double(0.01, 1.0, DEFAULTS["alpha"])

        parameters = [
            ("Threshold", self.threshold),
            ("Minimum area", self.min_area),
            ("Maximum area", self.max_area),
            ("Minimum aspect ratio", self.min_aspect),
            ("Maximum aspect ratio", self.max_aspect),
            ("Minimum fill ratio", self.min_fill),
            ("Maximum tracking jump (px)", self.max_jump),
            ("Smoothing alpha", self.alpha),
        ]

        for name, widget in parameters:
            form.addRow(name, widget)

        # Buttons
        self.apply_button = QPushButton("Apply now")
        self.reset_button = QPushButton("Reset defaults")
        self.save_button = QPushButton("Save settings")
        self.load_button = QPushButton("Load settings")

        buttons = QHBoxLayout()
        buttons.addWidget(self.apply_button)
        buttons.addWidget(self.reset_button)

        file_buttons = QHBoxLayout()
        file_buttons.addWidget(self.save_button)
        file_buttons.addWidget(self.load_button)

        # Layout
        preview_layout = QVBoxLayout()

        preview_layout.addWidget(QLabel("Camera preview"))
        preview_layout.addWidget(self.image_label, stretch=3)

        preview_layout.addWidget(QLabel("Binary threshold preview"))
        preview_layout.addWidget(self.threshold_label, stretch=2)

        preview_layout.addWidget(self.stats_label)

        preview_container = QWidget()
        preview_container.setLayout(preview_layout)

        tuning_layout = QVBoxLayout()

        tuning_layout.addWidget(controls)
        tuning_layout.addLayout(buttons)
        tuning_layout.addLayout(file_buttons)

        tuning_layout.addStretch()

        self.stop_button = QPushButton("Stop Calibration")

        tuning_layout.addWidget(self.stop_button)

        tuning_container = QWidget()
        tuning_container.setLayout(tuning_layout)

        body = QHBoxLayout(self)
        body.addWidget(preview_container, stretch=2)
        body.addWidget(tuning_container, stretch=1)

        # Signals
        for control in (
            self.threshold,
            self.min_area,
            self.max_area,
            self.min_aspect,
            self.max_aspect,
            self.min_fill,
            self.max_jump,
            self.alpha,
        ):
            control.valueChanged.connect(self._queue_send)

        self.apply_button.clicked.connect(
            self.send_config
        )

        self.reset_button.clicked.connect(
            self.reset_defaults
        )

        self.save_button.clicked.connect(
            self.save_settings
        )

        self.load_button.clicked.connect(
            self.load_settings
        )

        self.stop_button.clicked.connect(
            self.on_stop
        )

        # Timers
        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(33)
        self.refresh_timer.timeout.connect(self._refresh_frame)

        self.network_timer = QTimer(self)
        self.network_timer.setInterval(15)
        self.network_timer.timeout.connect(self._poll_tuning_socket)
        self.network_timer.start()

        # Debounced configuration transmission
        self.send_timer = QTimer(self)
        self.send_timer.setSingleShot(True)
        self.send_timer.setInterval(120)
        self.send_timer.timeout.connect(self.send_config)

        # Send initial configuration
        self.send_config()




    @staticmethod
    def _spin(minimum, maximum, value):
        widget = QSpinBox()
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        return widget

    @staticmethod
    def _double(minimum, maximum, value, step=0.01):
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(2)
        widget.setSingleStep(step)
        widget.setValue(value)
        return widget

    def current_settings(self):
        return {
            "threshold": self.threshold.value(),
            "min_area": self.min_area.value(),
            "max_area": self.max_area.value(),
            "min_aspect": self.min_aspect.value(),
            "max_aspect": self.max_aspect.value(),
            "min_fill": self.min_fill.value(),
            "max_jump": self.max_jump.value(),
            "alpha": self.alpha.value(),
        }

    # Runtime vision tuning
    def _queue_send(self, *_):
        if self.min_area.value() > self.max_area.value():
            self.tuning_status.setText("Invalid range: minimum area exceeds maximum area")
            return

        self.send_timer.start()

    def send_config(self):
        values = self.current_settings()

        if values["min_area"] > values["max_area"]:
            self.tuning_status.setText("Invalid area range")
            return

        self.sequence = (self.sequence + 1) & 0xFFFFFFFF

        packet = TUNING_PACKET.pack(
            b"VTN1",
            self.sequence,
            values["threshold"],
            values["min_area"],
            values["max_area"],
            values["min_aspect"],
            values["max_aspect"],
            values["min_fill"],
            values["max_jump"],
            values["alpha"],
        )

        try:
            self.tune_sock.sendto(packet, (config.CAMERA_HOST, config.VISION_TUNER_PORT))
            self.last_send_time = time.monotonic()
            self.tuning_status.setText(f"Sent configuration #{self.sequence}; waiting for ACK...")

        except OSError as exc:
            self.tuning_status.setText(f"Configuration send failed: {exc}")

    def _poll_tuning_socket(self):
        now = time.monotonic()

        for _ in range(32):
            try:
                packet, addr = self.tune_sock.recvfrom(128)

            except BlockingIOError:
                break

            except OSError:
                break

            if len(packet) != ACK_PACKET.size:
                continue

            magic, seq, status = ACK_PACKET.unpack(packet)

            if magic != b"ACK1":
                continue

            self.last_ack_seq = seq
            self.last_ack_time = now

            result = (
                "applied"
                if status == 0
                else "rejected (invalid parameters)"
            )

            self.tuning_status.setText(f"Camera ACK #{seq}: {result}")

        if (self.last_ack_seq is None and now - self.last_send_time > 2.0):
            self.tuning_status.setText("No ACK — check CAM IP, port 9006 and firmware tuner task")

    def reset_defaults(self):
        widgets = [
            (self.threshold, "threshold"),
            (self.min_area, "min_area"),
            (self.max_area, "max_area"),
            (self.min_aspect, "min_aspect"),
            (self.max_aspect, "max_aspect"),
            (self.min_fill, "min_fill"),
            (self.max_jump, "max_jump"),
            (self.alpha, "alpha"),
        ]

        for widget, name in widgets:
            widget.setValue(DEFAULTS[name])

        self.send_timer.stop()
        self.send_config()

    # Save / Load
    def save_settings(self):
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save vision settings",
            "vision_tuning.json",
            "JSON files (*.json)",
        )

        if not path:
            return

        try:
            Path(path).write_text(json.dumps(self.current_settings(), indent=2), encoding="utf-8")
            self.tuning_status.setText("Settings saved")

        except OSError as exc:
            QMessageBox.critical(self, "Could not save settings", str(exc))

    def load_settings(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Load vision settings",
            "",
            "JSON files (*.json)",
        )

        if not path:
            return

        try:
            values = json.loads(Path(path).read_text(encoding="utf-8"))
            merged = DEFAULTS | values

            self.threshold.setValue(int(merged["threshold"]))
            self.min_area.setValue(int(merged["min_area"]))
            self.max_area.setValue(int(merged["max_area"]))
            self.min_aspect.setValue(float(merged["min_aspect"]))
            self.max_aspect.setValue(float(merged["max_aspect"]))
            self.min_fill.setValue(float(merged["min_fill"]))
            self.max_jump.setValue(float(merged["max_jump"]))
            self.alpha.setValue(float(merged["alpha"]))

            self.send_timer.stop()
            self.send_config()

        except (OSError, ValueError, KeyError, TypeError) as exc:
            QMessageBox.critical(self, "Could not load settings", str(exc))

    # Camera preview
    def start_camera(self):
        if self.receiver is not None:
            return

        self.last_sequence = -1

        self.image_label.clear()
        self.image_label.setText("Waiting for camera...")

        self.threshold_label.clear()
        self.threshold_label.setText("Waiting for camera...")

        self.frame_status.setText("Connecting to camera...")

        self.receiver = FrameFetcher()

        self.receiver.connection_changed.connect(self._update_status)

        self.receiver.start()
        self.refresh_timer.start()

    def _update_status(self, message):
        message = message.lower()

        if "failed" in message or "error" in message:
            self.frame_status.setText("Camera connection error")

        elif "receiving" in message:
            self.frame_status.setText("Camera preview active")

        elif "waiting" in message:
            self.frame_status.setText("Waiting for camera")

        else:
            self.frame_status.setText("Connecting to camera...")

    def _refresh_frame(self):
        if self.receiver is None:
            return

        frame, sequence = self.receiver.get_latest_frame()

        if frame is None or sequence == self.last_sequence:
            return

        self.last_sequence = sequence

        pixels = np.frombuffer(frame, dtype=np.uint8).reshape(
            config.CAMERA_FRAME_HEIGHT,
            config.CAMERA_FRAME_WIDTH,
        )

        self._show_frame(pixels)

    def _show_frame(self, gray):
        height, width = gray.shape

        # Original grayscale camera image
        image = QImage(
            gray.data,
            width,
            height,
            gray.strides[0],
            QImage.Format.Format_Grayscale8,
        ).copy()

        self.image_label.setPixmap(
            QPixmap.fromImage(image).scaled(
                self.image_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
        )

        # Binary threshold preview
        crop = gray[CROP_TOP:CROP_BOTTOM, CROP_LEFT:CROP_RIGHT]
        binary = (crop > self.threshold.value()).astype(np.uint8) * 255
        white_pixels = int(np.count_nonzero(binary))
        binary_image = QImage(
            binary.data,
            CROP_WIDTH,
            CROP_HEIGHT,
            binary.strides[0],
            QImage.Format.Format_Grayscale8,
        ).copy()

        self.threshold_label.setPixmap(
            QPixmap.fromImage(binary_image).scaled(
                self.threshold_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
        )

        self.stats_label.setText(
            f"Frame: {self.last_sequence} | "
            f"White pixels: {white_pixels} | "
            f"Crop: {CROP_WIDTH} x {CROP_HEIGHT}"
        )

    # Telemetry from main window
    def set_ball_telemetry(self, sample):
        if sample is None:
            self.image_label.clear_ball_telemetry()

            self.telemetry_status.setText("Telemetry unavailable")
            return

        self.image_label.set_ball_telemetry(sample)

        if sample.ball_detected:
            self.telemetry_status.setText(f"Ball: ({sample.ball_x}, {sample.ball_y})"
            )
        else:
            self.telemetry_status.setText("Ball not detected")

    # Shutdown
    def stop_camera(self):
        self.refresh_timer.stop()

        receiver = self.receiver
        self.receiver = None

        if receiver is not None:
            receiver.stop()
            receiver.deleteLater()

        self.image_label.clear()
        self.image_label.setText("Camera preview stopped")
        self.threshold_label.clear()
        self.threshold_label.setText("Camera stopped")
        self.image_label.clear_ball_telemetry()
        self.frame_status.setText("Camera stopped")
        self.telemetry_status.setText("Telemetry unavailable")

    def closeEvent(self, event):
        self.stop_camera()
        self.network_timer.stop()
        self.send_timer.stop()
        self.tune_sock.close()
        event.accept()
