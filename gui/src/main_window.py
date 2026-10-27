import logging
import time

from PyQt6.QtCore import QTimer, QObject, pyqtSignal
from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget, QPushButton, QStackedWidget

import state

from communication.telemetry_receiver import (
    LinkMonitor,
    LinkState,
    TelemetryReceiver,
    TelemetrySample,
    ReceiverStats
)
from communication.esp_sender import ESPSender

from widgets.camera_calibration import CameraCalibrationPage
from widgets.plots import PlotsWidget
from widgets.telemetry import TelemetryWidget
from widgets.controller_panel import ControllerPanel

log = logging.getLogger(__name__)

def _log_telemetry_status(self, message: str):
    log.info("[TELEMETRY] %s", message)

class TelemetryBridge(QObject):
    sample_received = pyqtSignal(object, object)
    packet_rejected = pyqtSignal(str, object)
    receiver_failed = pyqtSignal(str)

class BallPlateWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Ball and Plate Control System")
        self.resize(1200, 750)

        self.telemetry = TelemetryWidget()
        self.plots = PlotsWidget()
        self.esp_sender = ESPSender();
        self.controller_panel = ControllerPanel(self.esp_sender)

        self.calibrate_button = QPushButton("Calibrate Plate")
        self.calibrate_button.clicked.connect(self.start_calibration)

        self.bridge = TelemetryBridge()
        self.link_monitor = LinkMonitor()
        self.latest_sample = None
        self.start_time = time.perf_counter()
        self.last_plotted_sequence = None

        # Main dashboard page
        self.dashboard_page = QWidget()
        self._setup_ui()

        # Camera calibration page
        self.calibration_page = CameraCalibrationPage(on_stop=self.stop_calibration)

        # Switch between dashboard and calibration
        self.page_stack = QStackedWidget()
        self.page_stack.addWidget(self.dashboard_page)
        self.page_stack.addWidget(self.calibration_page)

        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.addWidget(self.page_stack)

        # Camera receiver is managed by CameraCalibrationPage
        self._setup_receiver()

        # Keep dashboard telemetry and plots updating
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_dashboard)
        self.timer.start(33)

        self.link_timer = QTimer(self)
        self.link_timer.timeout.connect(self._update_link_status)
        self.link_timer.start(250)

    def _setup_ui(self):
        self.preview_header = QHBoxLayout()
        self.preview_header.addWidget(self.calibrate_button)
        self.preview_header.addStretch()

        left_layout = QVBoxLayout()
        left_layout.addLayout(self.preview_header)
        left_layout.addWidget(self.telemetry, stretch=3)
        left_layout.addWidget(self.controller_panel, stretch=3)

        main_layout = QHBoxLayout(self.dashboard_page)
        main_layout.addLayout(left_layout, stretch=2)
        main_layout.addWidget(self.plots, stretch=3)

    def _setup_receiver(self):
        self.bridge.sample_received.connect(self._on_sample)
        self.bridge.packet_rejected.connect(self._on_packet_rejected)
        self.bridge.receiver_failed.connect(self._on_receiver_failed)

        self.receiver = TelemetryReceiver(
            on_sample=lambda sample, stats: self.bridge.sample_received.emit(sample, stats),
            on_rejected=lambda reason, stats: self.bridge.packet_rejected.emit(reason, stats),
            on_failed=lambda message: self.bridge.receiver_failed.emit(message),
        )

        try:
            self.receiver.start()
            log.info("Telemetry receiver started on UDP port %s", self.receiver.port)
        except OSError as error:
            log.exception("Could not start telemetry receiver")

            with state.app_state._lock:
                state.app_state.connection.connected = False
                state.app_state.connection.last_error = str(error)

            self.telemetry.set_error("Unable to start connection")

    def _on_sample(self, sample: TelemetrySample, stats: ReceiverStats):
        """Runs on the Qt thread after the bridge emits a sample."""
        self.latest_sample = sample
        self.link_monitor.packet_received(time.monotonic())

        with state.app_state._lock:
            state.app_state.connection.connected = True
            state.app_state.connection.last_error = None
            state.app_state.ball.detected = sample.ball_detected
            state.app_state.ball.x = sample.ball_x
            state.app_state.ball.y = sample.ball_y
            state.app_state.ball.measurement_age_ms = sample.measurement_age_ms
            state.app_state.ball.sequence = sample.sequence

            state.app_state.pid.error_x = sample.error_x_mm
            state.app_state.pid.error_y = sample.error_y_mm
            state.app_state.pid.output_x_us = sample.output_x_us
            state.app_state.pid.output_y_us = sample.output_y_us

            state.app_state.controller_enabled = sample.controller_enabled

        self.telemetry.set_sample(sample)
        self.telemetry.set_stats(stats)
        self.telemetry.set_link_state(LinkState.LIVE)

        log.info(
            "[TELEMETRY] Received | seq=%s | detected=%s | "
            "ball=(%.1f, %.1f) px | age=%s ms | "
            "error=(%.2f, %.2f) mm | output=(%s, %s) us | "
            "controller=%s | received=%s rejected=%s missed=%s",
            sample.sequence,
            sample.ball_detected,
            sample.ball_x,
            sample.ball_y,
            sample.measurement_age_ms,
            sample.error_x_mm,
            sample.error_y_mm,
            sample.output_x_us,
            sample.output_y_us,
            sample.controller_enabled,
            stats.received,
            stats.rejected,
            stats.missed,
        )

    def _on_packet_rejected(self, reason: str, stats: ReceiverStats):
        log.warning(
            "Rejected telemetry packet: %s (received=%d, rejected=%d, missed=%d)",
            reason, stats.received, stats.rejected, stats.missed)

    def _on_receiver_failed(self, message: str):
        log.error("Telemetry receiver failed: %s", message)

        with state.app_state._lock:
            state.app_state.connection.connected = False
            state.app_state.connection.last_error = message

        self.telemetry.set_error("Connection error")

    def _update_dashboard(self):
        elapsed = time.perf_counter() - self.start_time

        sample = self.latest_sample

        if self.link_monitor.state(time.monotonic()) != LinkState.LIVE:
            return

        if sample is None or not sample.ball_detected:
            return

        if sample.sequence == self.last_plotted_sequence:
            return

        ball_px = (sample.ball_x, sample.ball_y)

        # TODO: convert from px to mm
        measured_mm = ball_px

        with state.app_state._lock:
            target_x = state.app_state.target.x
            target_y = state.app_state.target.y

        target_mm = (target_x, target_y)

        self.plots.add_sample(elapsed, measured_mm, target_mm)
        self.last_plotted_sequence = sample.sequence

    def _update_link_status(self):
        link_state = self.link_monitor.state(time.monotonic())

        self.telemetry.set_link_state(link_state)

        if link_state == LinkState.LIVE:
            with state.app_state._lock:
                state.app_state.connection.connected = True
        elif link_state in (LinkState.STALE, LinkState.NO_DATA):
            with state.app_state._lock:
                state.app_state.connection.connected = False

    def start_calibration(self):
        self.page_stack.setCurrentWidget(self.calibration_page)
        self.calibration_page.start_camera()

    def stop_calibration(self):
        self.calibration_page.stop_camera()
        self.page_stack.setCurrentWidget(self.dashboard_page)

    def closeEvent(self, event):
        """Stop timers and release the receiver socket on window close."""
        self.timer.stop()
        self.link_timer.stop()
        self.calibration_page.stop_camera()
        self.esp_sender.close()

        if hasattr(self, "receiver"):
            self.receiver.stop()

        event.accept()