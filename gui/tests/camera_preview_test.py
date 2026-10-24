import argparse
import time

import numpy as np
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap, QPainter, QPen, QColor
from PyQt6.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

from src.communication.camera_receiver import FrameFetcher
import src.config as config

WIDTH = config.CAMERA_FRAME_WIDTH
HEIGHT = config.CAMERA_FRAME
FRAME_SIZE = WIDTH * HEIGHT

DETECT_SIZE = config.CAMERA_FRAME_HEIGHT
CROP_X = (WIDTH - DETECT_SIZE) // 2

class CameraWindow(QMainWindow):
    def __init__(self, camera_ip):
        super().__init__()

        self.setWindowTitle("Ball and Plate - UDP Camera Preview")

        self.image_label = QLabel("Waiting for ESP32-CAM...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(320, 240)

        self.status_label = QLabel("Initializing UDP receiver...")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(self.image_label, 1)
        layout.addWidget(self.status_label)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.fetcher = FrameFetcher(camera_ip)
        self.fetcher.connection_changed.connect(self.on_connection_changed)
        self.fetcher.start()

        self.last_sequence = -1
        self.displayed_frames = 0
        self.fps_start = time.perf_counter()
        self.display_fps = 0.0

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_frame)
        self.timer.start(round(1000 / config.CAMERA_GUI_FPS))

    def on_connection_changed(self, message):
        self.status_label.setText(message)

    def refresh_frame(self):
        content, sequence = self.fetcher.get_latest_frame()

        if content is None or sequence == self.last_sequence:
            return

        self.last_sequence = sequence

        pixels = np.frombuffer(content,dtype=np.uint8).reshape(HEIGHT, WIDTH)

        image = QImage(
            pixels.data,
            WIDTH,
            HEIGHT,
            WIDTH,
            QImage.Format.Format_Grayscale8,
        ).copy()

        pixmap = QPixmap.fromImage(image).scaled(
            self.image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.FastTransformation,
        )

        # Draw the square detection region over the preview.
        scale_x = pixmap.width() / WIDTH
        scale_y = pixmap.height() / HEIGHT

        crop_left = round(CROP_X * scale_x)
        crop_top = 0
        crop_width = round(DETECT_SIZE * scale_x)
        crop_height = round(DETECT_SIZE * scale_y)

        painter = QPainter(pixmap)
        pen = QPen(QColor(0, 255, 0), 2)
        painter.setPen(pen)
        painter.drawRect(crop_left, crop_top, crop_width - 1, crop_height - 1)
        painter.end()

        self.image_label.setPixmap(pixmap)

        self.displayed_frames += 1
        elapsed = time.perf_counter() - self.fps_start

        if elapsed >= 1.0:
            self.display_fps = self.displayed_frames / elapsed
            self.displayed_frames = 0
            self.fps_start = time.perf_counter()

            packets, completed, incomplete = self.fetcher.get_statistics()

            print(f"Frame sequence: {sequence}")

            self.status_label.setText(
                f"UDP | Display: {self.display_fps:.1f} FPS "
                f"| Frame: {sequence} "
                f"| Complete: {completed} "
                f"| Incomplete: {incomplete}"
            )

    def closeEvent(self, event):
        self.timer.stop()
        self.fetcher.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Display UDP grayscale frames from the ESP32-CAM."
    )

    parser.add_argument(
        "--camera-ip",
        required=True,
        help=("ESP32-CAM station IP shown in its serial log, not the access-point IP."),
    )
    args = parser.parse_args()

    app = QApplication([])

    window = CameraWindow(args.camera_ip)
    window.resize(800, 600)
    window.show()

    app.exec()
