import argparse
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

ESP32_PORT = 9000
PC_PORT = 9001

WIDTH = 160
HEIGHT = 120
FRAME_SIZE = WIDTH * HEIGHT

HEADER_SIZE = 16
PAYLOAD_SIZE = 1200
PACKETS_PER_FRAME = (FRAME_SIZE + PAYLOAD_SIZE - 1) // PAYLOAD_SIZE

# GRAY: magic, sequence, frame length, packet index, packet count
HEADER_FORMAT = "!4sIIHH"

GUI_FPS = 30
SOCKET_TIMEOUT = 0.2
MAX_PENDING_FRAMES = 4
FRAME_EXPIRY_SECONDS = 0.25
HELLO_INTERVAL = 1.0


def is_newer_sequence(sequence, reference):
    """Compare wrapping uint32 sequence numbers."""
    if reference < 0:
        return True

    delta = (sequence - reference) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


class FrameFetcher(QThread):
    connection_changed = pyqtSignal(str)

    def __init__(self, camera_ip):
        super().__init__()

        self.camera_ip = camera_ip
        self.running = True
        self.sock = None
        self.lock = threading.Lock()

        self.latest_frame = None
        self.latest_sequence = -1

        self.received_packets = 0
        self.completed_frames = 0
        self.incomplete_frames = 0

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock = sock

        # Allow quick receiver restarts.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(SOCKET_TIMEOUT)

        try:
            sock.bind(("", PC_PORT))
        except OSError as exc:
            self.connection_changed.emit(f"UDP bind failed: {exc}")
            sock.close()
            self.sock = None
            return

        pending = {}

        last_hello = 0.0
        last_packet_time = time.perf_counter()
        last_status = ""

        self.connection_changed.emit(f"Listening on UDP port {PC_PORT}; registering camera...")

        try:
            while self.running:
                now = time.perf_counter()

                # Register the PC's receiving address with the camera.
                if now - last_hello >= HELLO_INTERVAL:
                    try:
                        sock.sendto(b"HELO", (self.camera_ip, ESP32_PORT))
                    except OSError as exc:
                        self.connection_changed.emit(f"UDP registration error: {exc}")

                    last_hello = now

                try:
                    packet, sender = sock.recvfrom(2048)
                except socket.timeout:
                    self._expire_frames(pending)
                    continue
                except OSError:
                    if self.running:
                        self.connection_changed.emit("UDP socket error")
                    break

                if sender[0] != self.camera_ip:
                    continue

                last_packet_time = time.perf_counter()

                if len(packet) < HEADER_SIZE:
                    continue

                (
                    magic,
                    sequence,
                    length,
                    packet_index,
                    packet_count,
                ) = struct.unpack(
                    HEADER_FORMAT,
                    packet[:HEADER_SIZE],
                )

                if magic != b"GRAY":
                    continue

                if length != FRAME_SIZE:
                    continue

                if packet_count != PACKETS_PER_FRAME:
                    continue

                if packet_index >= PACKETS_PER_FRAME:
                    continue

                offset = packet_index * PAYLOAD_SIZE
                expected_length = min(PAYLOAD_SIZE, FRAME_SIZE - offset)
                payload = packet[HEADER_SIZE:]

                if len(payload) != expected_length:
                    continue

                # Do not start reconstructing an already-obsolete frame.
                with self.lock:
                    latest_sequence = self.latest_sequence

                if not is_newer_sequence(sequence, latest_sequence):
                    continue

                frame = pending.get(sequence)

                if frame is None:
                    # Bound memory use by discarding the oldest pending frame.
                    while len(pending) >= MAX_PENDING_FRAMES:
                        oldest_sequence = min(pending, key=lambda seq: pending[seq]["time"])
                        pending.pop(oldest_sequence)
                        self.incomplete_frames += 1

                    frame = {
                        "pixels": bytearray(FRAME_SIZE),
                        "received": set(),
                        "time": time.perf_counter(),
                    }
                    pending[sequence] = frame

                # Ignore duplicate UDP packets.
                if packet_index not in frame["received"]:
                    frame["pixels"][offset:offset + expected_length] = payload
                    frame["received"].add(packet_index)
                    self.received_packets += 1

                # Publish complete frames only.
                if len(frame["received"]) == PACKETS_PER_FRAME:
                    completed = bytes(frame["pixels"])
                    pending.pop(sequence, None)

                    with self.lock:
                        if is_newer_sequence(sequence, self.latest_sequence):
                            self.latest_frame = completed
                            self.latest_sequence = sequence
                            self.completed_frames += 1

                self._expire_frames(pending)

                now = time.perf_counter()

                if now - last_packet_time > 2.0:
                    status = "Waiting for image packets..."
                else:
                    status = "Receiving UDP frames"

                if status != last_status:
                    self.connection_changed.emit(status)
                    last_status = status

        finally:
            sock.close()
            self.sock = None

    def _expire_frames(self, pending):
        now = time.perf_counter()

        expired = [
            sequence
            for sequence, frame in pending.items()
            if now - frame["time"] > FRAME_EXPIRY_SECONDS
        ]

        for sequence in expired:
            pending.pop(sequence, None)
            self.incomplete_frames += 1

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

        if self.isRunning():
            self.wait(2000)


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
        help=(
            "ESP32-CAM station IP shown in its serial log, "
            "not the access-point IP."
        ),
    )
    args = parser.parse_args()

    app = QApplication([])

    window = CameraWindow(args.camera_ip)
    window.resize(800, 600)
    window.show()

    app.exec()
