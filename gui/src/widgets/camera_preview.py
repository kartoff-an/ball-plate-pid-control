"""Camera preview panel with ball and target overlays.

The panel only displays data it is given. It does not capture frames or
detect the ball itself, so the same widget works with mock frames now and
the ESP32-CAM stream later.
"""

import numpy as np
from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget

BALL_COLOR = QColor(255, 80, 80)
TARGET_COLOR = QColor(80, 200, 120)
NO_SIGNAL_TEXT = "No camera signal"


class _FrameView(QWidget):
    """Draws the frame scaled to fit, preserving its aspect ratio."""

    def __init__(self, frame_width, frame_height, parent=None):
        super().__init__(parent)
        self._frame_width = frame_width
        self._frame_height = frame_height
        self._image = None
        self.ball = None
        self.target = None

        self.setMinimumSize(frame_width * 2, frame_height * 2)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_image(self, image):
        self._image = image
        self.update()

    def _image_rect(self):
        """Largest rectangle with the frame's aspect ratio that fits the widget."""
        scale = min(self.width() / self._frame_width, self.height() / self._frame_height)
        w = self._frame_width * scale
        h = self._frame_height * scale
        return QRectF((self.width() - w) / 2, (self.height() - h) / 2, w, h)

    def _to_widget(self, rect, point):
        x, y = point
        return QPointF(
            rect.x() + x * rect.width() / self._frame_width,
            rect.y() + y * rect.height() / self._frame_height,
        )

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(20, 20, 20))
        rect = self._image_rect()

        if self._image is None:
            painter.setPen(QColor(150, 150, 150))
            painter.drawRect(rect)
            painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, NO_SIGNAL_TEXT)
            return

        painter.drawImage(rect, self._image)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Marker size follows the display scale so it stays proportional.
        size = max(6.0, rect.width() / self._frame_width * 6)

        if self.target is not None:
            center = self._to_widget(rect, self.target)
            painter.setPen(QPen(TARGET_COLOR, 2))
            painter.drawLine(center + QPointF(-size, 0), center + QPointF(size, 0))
            painter.drawLine(center + QPointF(0, -size), center + QPointF(0, size))

        if self.ball is not None:
            center = self._to_widget(rect, self.ball)
            painter.setPen(QPen(BALL_COLOR, 2))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(center, size, size)


class CameraPreviewWidget(QWidget):
    """Camera feed with ball/target markers and their image coordinates.

    Positions are in image pixels, with (0, 0) at the top-left of the frame.
    Pass None for a position that is currently unavailable.
    """

    def __init__(self, frame_width=160, frame_height=120, parent=None):
        super().__init__(parent)
        self._view = _FrameView(frame_width, frame_height)

        self._status = QLabel(NO_SIGNAL_TEXT)
        self._ball_label = QLabel()
        self._target_label = QLabel()

        mono = QFont("Consolas")
        mono.setStyleHint(QFont.StyleHint.Monospace)
        for label, color in ((self._ball_label, BALL_COLOR), (self._target_label, TARGET_COLOR)):
            label.setFont(mono)
            label.setStyleSheet(f"color: {color.name()};")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._status)
        layout.addWidget(self._view, stretch=1)
        layout.addWidget(self._ball_label)
        layout.addWidget(self._target_label)

        self._refresh_labels()

    def set_frame(self, frame):
        """Show a grayscale uint8 frame of shape (height, width), or None to clear it."""
        if frame is None:
            self._view.set_image(None)
            self._status.setText(NO_SIGNAL_TEXT)
            return

        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        height, width = frame.shape
        # copy() so the QImage owns its pixels instead of borrowing the array's buffer.
        image = QImage(frame.data, width, height, width, QImage.Format.Format_Grayscale8).copy()
        self._view.set_image(image)

    def set_status(self, text):
        self._status.setText(text)

    def set_ball_position(self, position):
        self._view.ball = position
        self._refresh_labels()

    def set_target_position(self, position):
        self._view.target = position
        self._refresh_labels()

    def _refresh_labels(self):
        self._ball_label.setText(_format_position("Ball  ", self._view.ball))
        self._target_label.setText(_format_position("Target", self._view.target))
        self._view.update()


def _format_position(name, position):
    # Show "--" rather than 0 so missing data isn't mistaken for a real reading.
    if position is None:
        return f"{name}  X: --     Y: --     px"
    x, y = position
    return f"{name}  X: {x:6.1f} Y: {y:6.1f} px"


if __name__ == "__main__":
    # Standalone demo with mock frames. From gui/: python -m widgets.camera_preview
    import sys
    import time

    from PyQt6.QtCore import QTimer
    from PyQt6.QtWidgets import QApplication

    from mock_data import FRAME_HEIGHT, FRAME_WIDTH, MockCamera

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    camera = MockCamera()
    preview = CameraPreviewWidget(FRAME_WIDTH, FRAME_HEIGHT)
    preview.setWindowTitle("Camera Preview (mock data)")
    preview.resize(640, 540)
    preview.set_status("Mock camera: 160x120 grayscale")
    preview.set_target_position((FRAME_WIDTH / 2, FRAME_HEIGHT / 2))

    start = time.perf_counter()

    def tick():
        t = time.perf_counter() - start
        preview.set_frame(camera.frame(t))
        preview.set_ball_position(camera.ball_position(t))

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(33)

    preview.show()
    sys.exit(app.exec())
