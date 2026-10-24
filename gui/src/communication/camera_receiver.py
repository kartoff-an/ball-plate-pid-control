
import socket
import struct
import threading
import time

from PyQt6.QtCore import QThread, pyqtSignal

import config


HEADER_FORMAT = "!4sIIHH"
FRAME_MAGIC = b"GRAY"


def is_newer_sequence(sequence: int, reference: int) -> bool:
    """Compare wrapping unsigned 32-bit sequence numbers."""
    if reference < 0:
        return True

    delta = (sequence - reference) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


class FrameFetcher(QThread):
    connection_changed = pyqtSignal(str)

    def __init__(self, camera_ip=None):
        super().__init__()

        self.camera_ip = camera_ip or config.CAMERA_HOST
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

        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(config.CAMERA_SOCKET_TIMEOUT_S)

        try:
            sock.bind(("", config.PC_CAMERA_PORT))
        except OSError as exc:
            self.connection_changed.emit(f"UDP bind failed: {exc}")
            sock.close()
            self.sock = None
            return

        pending = {}
        last_hello = 0.0
        last_packet_time = time.perf_counter()
        last_status = ""

        self.connection_changed.emit("Registering camera...")

        try:
            while self.running:
                now = time.perf_counter()

                # Register the PC's receiving address with the camera.
                if now - last_hello >= config.CAMERA_HELLO_INTERVAL_S:
                    try:
                        sock.sendto(b"HELO", (self.camera_ip, config.CAMERA_UDP_PORT))
                    except OSError as exc:
                        self.connection_changed.emit(f"UDP registration error: {exc}")

                    last_hello = now

                try:
                    packet, sender = sock.recvfrom(2048)
                except socket.timeout:
                    self._expire_frames(pending)

                    if time.perf_counter() - last_packet_time > 2.0:
                        status = "Waiting for image packets..."
                        if status != last_status:
                            self.connection_changed.emit(status)
                            last_status = status

                    continue
                except OSError:
                    if self.running:
                        self.connection_changed.emit("UDP socket error")
                    break

                if sender[0] != self.camera_ip:
                    continue

                last_packet_time = time.perf_counter()

                if len(packet) < config.CAMERA_HEADER_SIZE:
                    continue

                (
                    magic,
                    sequence,
                    length,
                    packet_index,
                    packet_count,
                ) = struct.unpack(
                    HEADER_FORMAT,
                    packet[:config.CAMERA_HEADER_SIZE],
                )

                if magic != FRAME_MAGIC:
                    continue

                if length != config.CAMERA_FRAME_SIZE:
                    continue

                if packet_count != config.CAMERA_PACKETS_PER_FRAME:
                    continue

                if packet_index >= config.CAMERA_PACKETS_PER_FRAME:
                    continue

                offset = packet_index * config.CAMERA_PAYLOAD_SIZE
                expected_length = min(config.CAMERA_PAYLOAD_SIZE, config.CAMERA_FRAME_SIZE - offset)
                payload = packet[config.CAMERA_HEADER_SIZE:]

                if len(payload) != expected_length:
                    continue

                with self.lock:
                    latest_sequence = self.latest_sequence

                # Ignore frames that are not newer than the displayed frame.
                if not is_newer_sequence(sequence, latest_sequence):
                    continue

                frame = pending.get(sequence)

                if frame is None:
                    # Bound memory use by dropping the oldest incomplete frame.
                    while len(pending) >= config.CAMERA_MAX_PENDING_FRAMES:
                        oldest_sequence = min(pending, key=lambda seq: pending[seq]["time"])
                        pending.pop(oldest_sequence)
                        self.incomplete_frames += 1

                    frame = {
                        "pixels": bytearray(config.CAMERA_FRAME_SIZE),
                        "received": set(),
                        "time": time.perf_counter(),
                    }
                    pending[sequence] = frame

                # Ignore duplicate packets.
                if packet_index not in frame["received"]:
                    frame["pixels"][offset:offset + expected_length] = payload
                    frame["received"].add(packet_index)
                    self.received_packets += 1

                # Publish complete frames only.
                if len(frame["received"]) == config.CAMERA_PACKETS_PER_FRAME:
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
            if now - frame["time"] > config.CAMERA_FRAME_EXPIRY_S
        ]

        for sequence in expired:
            pending.pop(sequence, None)
            self.incomplete_frames += 1

    def get_latest_frame(self):
        with self.lock:
            return self.latest_frame, self.latest_sequence

    def get_statistics(self):
        with self.lock:
            return (self.received_packets, self.completed_frames, self.incomplete_frames)

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
