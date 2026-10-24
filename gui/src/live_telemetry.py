"""Shows live telemetry from the ESP32-S3 in the telemetry panel.

Join the BallPlateSystem Wi-Fi network first, then from gui/:

    python -m live_telemetry [--port PORT] [--stale-timeout SECONDS]

The defaults come from config.py.

This only listens. It sends nothing to the firmware.
"""

import argparse
import logging
import sys
import time

from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from PyQt6.QtWidgets import QApplication

import config
from state import app_state
from communication.telemetry_receiver import LinkMonitor, TelemetryReceiver
from widgets.telemetry import TelemetryWidget

# How often the link status is re-checked for staleness.
STATUS_REFRESH_MS = 200


class TelemetryBridge(QObject):
    """Carries receiver callbacks onto the GUI thread.

    The receiver calls these from its own thread. Emitting a signal whose
    receiver lives on the GUI thread queues the call there, so widgets are
    only ever touched from the GUI thread.
    """

    sample_received = pyqtSignal(object, object)
    packet_rejected = pyqtSignal(str, object)
    receiver_failed = pyqtSignal(str)


class LiveTelemetryController(QObject):
    """Connects a TelemetryReceiver to a TelemetryWidget."""

    def __init__(
        self,
        widget,
        port=config.PC_TELEMETRY_PORT,
        stale_timeout=config.TELEMETRY_STALE_TIMEOUT_S,
        host="",
        parent=None,
    ):
        super().__init__(parent)
        self._widget = widget
        self._port = port
        self._monitor = LinkMonitor(stale_timeout)
        self._failed = False

        self._bridge = TelemetryBridge()
        self._bridge.sample_received.connect(self._on_sample)
        self._bridge.packet_rejected.connect(self._on_rejected)
        self._bridge.receiver_failed.connect(self._on_failed)

        self._receiver = TelemetryReceiver(
            on_sample=self._bridge.sample_received.emit,
            on_rejected=self._bridge.packet_rejected.emit,
            on_failed=self._bridge.receiver_failed.emit,
            port=port,
            host=host,
        )

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._refresh_status)

    def start(self):
        try:
            self._receiver.start()
        except OSError as error:
            self._on_failed(f"Cannot listen on UDP port {self._port}: {error}")
            return False

        self._refresh_status()
        self._status_timer.start(STATUS_REFRESH_MS)
        return True

    def stop(self):
        self._status_timer.stop()
        self._receiver.stop()

    def _on_sample(self, sample, stats):
        self._monitor.packet_received(sample.received_at)
        _store_in_app_state(sample)
        self._widget.set_sample(sample)
        self._widget.set_stats(stats)
        self._refresh_status()

    def _on_rejected(self, reason, stats):
        self._widget.set_stats(stats)
        self._widget.set_last_rejection(reason)

    def _on_failed(self, message):
        logging.getLogger(__name__).error(message)
        self._failed = True
        self._status_timer.stop()
        self._widget.set_error(message)

    def _refresh_status(self):
        if self._failed:
            return
        now = time.monotonic()
        self._widget.set_link_state(
            self._monitor.state(now),
            self._monitor.seconds_since_last(now),
            self._monitor.stale_timeout,
        )


def _store_in_app_state(sample):
    """Copy the latest measurement into the shared app state for other parts of the GUI."""
    with app_state._lock:
        ball = app_state.ball
        ball.detected = sample.ball_detected
        ball.x = sample.ball_x
        ball.y = sample.ball_y
        ball.measurement_age_ms = sample.measurement_age_ms
        ball.sequence = sample.sequence

        pid = app_state.pid
        pid.error_x = sample.error_x_mm
        pid.error_y = sample.error_y_mm
        pid.output_x_us = sample.output_x_us
        pid.output_y_us = sample.output_y_us


def main():
    parser = argparse.ArgumentParser(description="Live ESP32-S3 telemetry monitor")
    parser.add_argument("--port", type=int, default=config.PC_TELEMETRY_PORT)
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=config.TELEMETRY_STALE_TIMEOUT_S,
        help="seconds without a packet before telemetry is marked stale",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    widget = TelemetryWidget()
    widget.setWindowTitle(f"Live Telemetry (UDP {args.port})")
    widget.resize(380, 520)

    controller = LiveTelemetryController(widget, port=args.port, stale_timeout=args.stale_timeout)
    app.aboutToQuit.connect(controller.stop)
    controller.start()

    widget.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
