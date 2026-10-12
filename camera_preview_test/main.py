
import socket
import struct
import threading
import time

import numpy as np
from PyQt6.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

HOST = "192.168.4.1"
ESP32_PORT = 9000
PC_PORT = 9001

WIDTH = 160
HEIGHT = 120
FRAME_SIZE = WIDTH * HEIGHT

# Must match the ESP32 firmware.
HEADER_SIZE = 16
PAYLOAD_SIZE = 1200
PACKETS_PER_FRAME = (FRAME_SIZE + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE

GUI_FPS = 30
SOCKET_TIMEOUT = 0.2
MAX_PENDING_FRAMES = 4
HELLO_INTERVAL = 1.0

# Header:
# 4 bytes magic
# 4 bytes sequence
# 4 bytes frame length
# 2 bytes packet index
# 2 bytes packet count
HEADER_FORMAT = "!4sIIHH"


class FrameFetcher(QThread):
    connection_changed = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self.running = True
        self.sock = None
        self.lock = threading.Lock()

        self.latest_frame = None
        self.latest_sequence = -1

        self.received_packets = 0
        self.incomplete_frames = 0
        self.completed_frames = 0

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock = sock

        # Allow the PC receiver to be restarted without waiting for
        # the previous socket's address state to expire.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(SOCKET_TIMEOUT)

        try:
            sock.bind(("", PC_PORT))
        except OSError as exc:
            self.connection_changed.emit(f"UDP bind failed: {exc}")
            sock.close()
            self.sock = None
            return

        # sequence -> {"pixels": bytearray, "received": set, "time": float}
        pending = {}

        last_hello = 0.0
        last_packet_time = 0.0

        self.connection_changed.emit("Waiting for ESP32-CAM...")

        try:
            while self.running:
                now = time.perf_counter()

                # Register periodically. This also helps recover if the
                # initial registration datagram was lost.
                if now - last_hello >= HELLO_INTERVAL:
                    try:
                        sock.sendto(b"HELO", (HOST, ESP32_PORT))
                    except OSError as exc:
                        self.connection_changed.emit(
                            f"UDP registration error: {exc}"
                        )

                    last_hello = now

                try:
                    packet, sender = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                except OSError:
                    if self.running:
                        self.connection_changed.emit("UDP socket error")
                    break

                last_packet_time = time.perf_counter()

                if sender[0] != HOST:
                    continue

                if len(packet) < HEADER_SIZE:
                    continue

                magic, sequence, length, packet_index, packet_count = (
                    struct.unpack(
                        HEADER_FORMAT,
                        packet[:HEADER_SIZE],
                    )
                )

                if magic != b"GRAY":
                    continue

                if length != FRAME_SIZE:
                    continue

                if packet_count != PACKETS_PER_FRAME:
                    continue

                if packet_index >= PACKETS_PER_FRAME:
                    continue

                payload = packet[HEADER_SIZE:]

                offset = packet_index * PAYLOAD_SIZE
                expected_length = min(
                    PAYLOAD_SIZE,
                    FRAME_SIZE - offset,
                )

                if len(payload) != expected_length:
                    continue

                # Ignore frames that are not newer than the last displayed
                # complete frame. Sequence numbers are uint32.
                with self.lock:
                    last_sequence = self.latest_sequence

                if last_sequence >= 0:
                    delta = (sequence - last_sequence) & 0xFFFFFFFF

                    if delta == 0 or delta >= 0x80000000:
                        continue

                frame = pending.get(sequence)

                if frame is None:
                    # Bound memory use and prevent old incomplete frames
                    # from accumulating.
                    while len(pending) >= MAX_PENDING_FRAMES:
                        oldest_sequence = min(
                            pending,
                            key=lambda seq: pending[seq]["time"],
                        )
                        pending.pop(oldest_sequence)
                        self.incomplete_frames += 1

                    frame = {
                        "pixels": bytearray(FRAME_SIZE),
                        "received": set(),
                        "time": time.perf_counter(),
                    }
                    pending[sequence] = frame

                # Ignore duplicate datagrams.
                if packet_index not in frame["received"]:
                    frame["pixels"][
                        offset:offset + expected_length
                    ] = payload

                    frame["received"].add(packet_index)
                    self.received_packets += 1

                # Publish only complete frames.
                if len(frame["received"]) == PACKETS_PER_FRAME:
                    completed = bytes(frame["pixels"])
                    pending.pop(sequence, None)

                    with self.lock:
                        current = self.latest_sequence

                        is_newer = (
                            current < 0
                            or 0 < ((sequence - current) & 0xFFFFFFFF)
                            < 0x80000000
                        )

                        if is_newer:
                            self.latest_frame = completed
                            self.latest_sequence = sequence
                            self.completed_frames += 1

                # Expire incomplete frames after 250 ms.
                now = time.perf_counter()

                expired = [
                    seq
                    for seq, item in pending.items()
                    if now - item["time"] > 0.25
                ]

                for seq in expired:
                    pending.pop(seq, None)
                    self.incomplete_frames += 1

                # Update connection status if frames stop arriving.
                if now - last_packet_time > 2.0:
                    self.connection_changed.emit(
                        "Waiting for image packets..."
                    )

        finally:
            sock.close()
            self.sock = None

    def get_latest_frame(self):
        with self.lock:
            return self.latest_frame, self.latest_sequence

    def get_statistics(self):
        with self.lock:
            return (
                self.received_packets,
                self.completed_frames,
                self.incomplete_frames,
            )

    def stop(self):
        self.running = False

        sock = self.sock
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

        self.wait(2000)


class CameraWindow(QMainWindow):
    def __init__(self):
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

        self.fetcher = FrameFetcher()
        self.fetcher.connection_changed.connect(
            self.on_connection_changed
        )
        self.fetcher.start()

        self.last_sequence = -1
        self.displayed_frames = 0
        self.fps_start = time.perf_counter()
        self.display_fps = 0.0

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_frame)
        self.timer.start(round(1000 / GUI_FPS))

    def on_connection_changed(self, message):
        self.status_label.setText(message)

    def refresh_frame(self):
        content, sequence = self.fetcher.get_latest_frame()

        if content is None or sequence == self.last_sequence:
            return

        self.last_sequence = sequence

        pixels = np.frombuffer(
            content,
            dtype=np.uint8,
        ).reshape(HEIGHT, WIDTH)

        # Copy pixels so the QImage owns independent image data.
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

        self.image_label.setPixmap(pixmap)

        self.displayed_frames += 1
        elapsed = time.perf_counter() - self.fps_start

        if elapsed >= 1.0:
            self.display_fps = self.displayed_frames / elapsed
            self.displayed_frames = 0
            self.fps_start = time.perf_counter()

            packets, completed, incomplete = (
                self.fetcher.get_statistics()
            )

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
    app = QApplication([])

    window = CameraWindow()
    window.resize(800, 600)
    window.show()

    app.exec()
