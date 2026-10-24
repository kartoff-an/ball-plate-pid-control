import numpy as np

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
)

import config
from communication.camera_receiver import FrameFetcher


class CameraCalibrationPage(QWidget):
    def __init__(self, on_stop):
        super().__init__()

        self.on_stop = on_stop
        self.receiver = None
        self.last_sequence = -1

        self.image_label = QLabel("Waiting for camera...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setStyleSheet("background-color: #111; color: white;")

        self.status_label = QLabel("Camera stopped")

        self.stop_button = QPushButton("Stop Calibration")
        self.stop_button.clicked.connect(self.on_stop)

        controls = QHBoxLayout()
        controls.addWidget(self.status_label)
        controls.addStretch()
        controls.addWidget(self.stop_button)

        layout = QVBoxLayout(self)
        layout.addWidget(self.image_label, stretch=1)
        layout.addLayout(controls)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(33)
        self.refresh_timer.timeout.connect(self._refresh_frame)

    def start_camera(self):
        if self.receiver is not None:
            return

        self.last_sequence = -1
        self.image_label.setText("Waiting for camera...")
        self.status_label.setText("Connecting to camera...")

        self.receiver = FrameFetcher()
        self.receiver.connection_changed.connect(self._update_status)
        self.receiver.start()

        self.refresh_timer.start()

    def _update_status(self, message):
        message = message.lower()

        if "failed" in message or "error" in message:
            self.status_label.setText("Camera connection error")
        elif "receiving" in message:
            self.status_label.setText("Camera preview active")
        elif "waiting" in message:
            self.status_label.setText("Waiting for camera")
        else:
            self.status_label.setText("Connecting to camera...")

    def _refresh_frame(self):
        if self.receiver is None:
            return

        frame, sequence = self.receiver.get_latest_frame()

        if frame is None or sequence == self.last_sequence:
            return

        self.last_sequence = sequence

        pixels = np.frombuffer(frame, dtype=np.uint8).reshape(config.CAMERA_FRAME_HEIGHT, config.CAMERA_FRAME_WIDTH)

        height, width = pixels.shape

        image = QImage(
            pixels.data,
            width,
            height,
            pixels.strides[0],
            QImage.Format.Format_Grayscale8,
        ).copy()

        pixmap = QPixmap.fromImage(image)

        self.image_label.setPixmap(
            pixmap.scaled(
                self.image_label.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
        )

    def stop_camera(self):
        self.refresh_timer.stop()

        receiver = self.receiver
        self.receiver = None

        if receiver is not None:
            receiver.stop()
            receiver.deleteLater()

        self.image_label.clear()
        self.image_label.setText("Camera preview stopped")
        self.status_label.setText("Camera stopped")
