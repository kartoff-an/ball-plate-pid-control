"""Tests for telemetry parsing, freshness tracking, and the UDP receiver.

From gui/: python -m unittest discover -s tests -v
"""

import json
import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from telemetry_receiver import (  # noqa: E402
    LinkMonitor,
    LinkState,
    SequenceTracker,
    TelemetryParseError,
    TelemetryReceiver,
    parse_telemetry,
)

# Exactly the format telemetry.c produces.
FIRMWARE_PACKET = (
    b'{"type":"telemetry","sequence":42,"ball_detected":true,"ball_x":80,"ball_y":60,'
    b'"measurement_age_ms":12,"error_x_mm":-3.50,"error_y_mm":7.00,"output_x_us":-1.75,'
    b'"output_y_us":3.50,"servo_x_us":1498,"servo_y_us":1503,"controller_enabled":false}'
)


def packet(**overrides):
    payload = json.loads(FIRMWARE_PACKET)
    for name, value in overrides.items():
        if value is _REMOVE:
            del payload[name]
        else:
            payload[name] = value
    return json.dumps(payload).encode("utf-8")


_REMOVE = object()


class ParseTelemetryTests(unittest.TestCase):
    def test_parses_firmware_packet(self):
        sample = parse_telemetry(FIRMWARE_PACKET, received_at=12.5)

        self.assertEqual(sample.sequence, 42)
        self.assertIs(sample.ball_detected, True)
        self.assertEqual((sample.ball_x, sample.ball_y), (80, 60))
        self.assertEqual(sample.measurement_age_ms, 12)
        self.assertEqual((sample.error_x_mm, sample.error_y_mm), (-3.5, 7.0))
        self.assertEqual((sample.output_x_us, sample.output_y_us), (-1.75, 3.5))
        self.assertEqual((sample.servo_x_us, sample.servo_y_us), (1498, 1503))
        self.assertIs(sample.controller_enabled, False)
        self.assertEqual(sample.received_at, 12.5)

    def test_integer_valued_numbers_become_floats(self):
        sample = parse_telemetry(packet(error_x_mm=0, output_y_us=5), received_at=0)
        self.assertIsInstance(sample.error_x_mm, float)
        self.assertIsInstance(sample.output_y_us, float)

    def test_ignores_unknown_extra_fields(self):
        sample = parse_telemetry(packet(future_field="anything"), received_at=0)
        self.assertEqual(sample.sequence, 42)

    def test_accepts_largest_sequence(self):
        sample = parse_telemetry(packet(sequence=2**32 - 1), received_at=0)
        self.assertEqual(sample.sequence, 2**32 - 1)

    def assertRejected(self, datagram, message_part):
        with self.assertRaises(TelemetryParseError) as caught:
            parse_telemetry(datagram, received_at=0)
        self.assertIn(message_part, str(caught.exception))

    def test_rejects_malformed_json(self):
        self.assertRejected(b'{"type":"telemetry",', "malformed JSON")
        self.assertRejected(b"", "malformed JSON")

    def test_rejects_non_utf8(self):
        self.assertRejected(b"\xff\xfe{}", "not UTF-8")

    def test_rejects_non_object(self):
        self.assertRejected(b"[1, 2, 3]", "not a JSON object")

    def test_rejects_unrelated_packet_types(self):
        self.assertRejected(packet(type="detection"), "unrelated packet type 'detection'")
        self.assertRejected(packet(type=_REMOVE), "unrelated packet type None")
        self.assertRejected(b"HELO", "malformed JSON")

    def test_rejects_missing_fields(self):
        self.assertRejected(packet(servo_y_us=_REMOVE), "missing fields: servo_y_us")
        # The firmware sends *_mm, not bare error_x/error_y.
        self.assertRejected(
            packet(error_x_mm=_REMOVE, error_y_mm=_REMOVE, error_x=1.0, error_y=1.0),
            "missing fields: error_x_mm, error_y_mm",
        )

    def test_rejects_numbers_sent_as_strings(self):
        self.assertRejected(packet(ball_x="80"), "ball_x must be a integer")
        self.assertRejected(packet(error_x_mm="1.00"), "error_x_mm must be a number")

    def test_rejects_wrong_scalar_types(self):
        self.assertRejected(packet(ball_detected=1), "ball_detected must be a boolean")
        self.assertRejected(packet(controller_enabled="false"), "controller_enabled must be a boolean")
        self.assertRejected(packet(sequence=True), "sequence must be a integer")
        self.assertRejected(packet(servo_x_us=1500.5), "servo_x_us must be a integer")
        self.assertRejected(packet(output_x_us=None), "output_x_us must be a number")

    def test_rejects_out_of_range_values(self):
        self.assertRejected(packet(ball_y=-1), "ball_y must not be negative")
        self.assertRejected(packet(sequence=2**32), "sequence exceeds 32 bits")

    def test_rejects_non_finite_numbers(self):
        datagram = FIRMWARE_PACKET.replace(b'"error_x_mm":-3.50', b'"error_x_mm":NaN')
        self.assertRejected(datagram, "non-finite")

    def test_rejects_oversized_datagram(self):
        self.assertRejected(b" " * 4096, "too large")


class SequenceTrackerTests(unittest.TestCase):
    def test_counts_gaps(self):
        tracker = SequenceTracker()
        self.assertEqual(tracker.update(10), 0)
        self.assertEqual(tracker.update(11), 0)
        self.assertEqual(tracker.update(15), 3)
        self.assertEqual(tracker.missed, 3)

    def test_handles_uint32_wraparound(self):
        tracker = SequenceTracker()
        tracker.update(2**32 - 2)
        self.assertEqual(tracker.update(2**32 - 1), 0)
        self.assertEqual(tracker.update(0), 0)
        self.assertEqual(tracker.update(2), 1)
        self.assertEqual(tracker.missed, 1)

    def test_backwards_jump_is_a_restart_not_a_loss(self):
        tracker = SequenceTracker()
        tracker.update(5000)
        self.assertEqual(tracker.update(0), 0)
        self.assertEqual(tracker.missed, 0)
        self.assertEqual(tracker.restarts, 1)

    def test_duplicate_is_not_a_loss(self):
        tracker = SequenceTracker()
        tracker.update(7)
        self.assertEqual(tracker.update(7), 0)
        self.assertEqual(tracker.missed, 0)


class LinkMonitorTests(unittest.TestCase):
    def test_no_data_before_first_packet(self):
        monitor = LinkMonitor(stale_timeout=1.0)
        self.assertIs(monitor.state(now=100.0), LinkState.NO_DATA)
        self.assertIsNone(monitor.seconds_since_last(now=100.0))

    def test_live_within_timeout_then_stale(self):
        monitor = LinkMonitor(stale_timeout=1.0)
        monitor.packet_received(now=10.0)

        self.assertIs(monitor.state(now=10.0), LinkState.LIVE)
        self.assertIs(monitor.state(now=11.0), LinkState.LIVE)
        self.assertIs(monitor.state(now=11.01), LinkState.STALE)
        self.assertAlmostEqual(monitor.seconds_since_last(now=13.0), 3.0)

    def test_new_packet_makes_stale_link_live_again(self):
        monitor = LinkMonitor(stale_timeout=0.5)
        monitor.packet_received(now=1.0)
        self.assertIs(monitor.state(now=5.0), LinkState.STALE)

        monitor.packet_received(now=5.0)
        self.assertIs(monitor.state(now=5.1), LinkState.LIVE)


class Collector:
    """Records receiver callbacks and lets a test wait for them."""

    def __init__(self):
        self.samples = []
        self.rejections = []
        self.stats = None
        self._changed = threading.Condition()

    def on_sample(self, sample, stats):
        with self._changed:
            self.samples.append(sample)
            self.stats = stats
            self._changed.notify_all()

    def on_rejected(self, reason, stats):
        with self._changed:
            self.rejections.append(reason)
            self.stats = stats
            self._changed.notify_all()

    def wait_for(self, condition, timeout=2.0):
        with self._changed:
            if not self._changed.wait_for(condition, timeout):
                raise AssertionError("timed out waiting for receiver callbacks")


class TelemetryReceiverTests(unittest.TestCase):
    def setUp(self):
        self.collector = Collector()
        # Port 0 picks a free port, so this never clashes with a real receiver.
        self.receiver = TelemetryReceiver(
            on_sample=self.collector.on_sample,
            on_rejected=self.collector.on_rejected,
            port=0,
            host="127.0.0.1",
        )
        self.receiver.start()
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.sender.close()
        self.receiver.stop()

    def send(self, datagram):
        self.sender.sendto(datagram, ("127.0.0.1", self.receiver.port))

    def test_receives_valid_and_survives_invalid_datagrams(self):
        self.send(packet(sequence=1))
        self.send(b"not json")
        self.send(packet(type="other"))
        self.send(packet(sequence=4))

        self.collector.wait_for(lambda: len(self.collector.samples) == 2 and len(self.collector.rejections) == 2)

        self.assertEqual([s.sequence for s in self.collector.samples], [1, 4])
        self.assertEqual(self.collector.stats.received, 2)
        self.assertEqual(self.collector.stats.rejected, 2)
        self.assertEqual(self.collector.stats.missed, 2)

    def test_stamps_samples_with_local_receive_time(self):
        before = time.monotonic()
        self.send(FIRMWARE_PACKET)
        self.collector.wait_for(lambda: self.collector.samples)
        self.assertGreaterEqual(self.collector.samples[0].received_at, before)
        self.assertLessEqual(self.collector.samples[0].received_at, time.monotonic())

    def test_stop_joins_thread_and_releases_port(self):
        port = self.receiver.port
        self.receiver.stop()
        self.receiver.stop()  # second call is harmless

        self.assertFalse(any(t.name == "telemetry-receiver" and t.is_alive() for t in threading.enumerate()))

        # Binding the same port again only works if the socket was closed.
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("127.0.0.1", port))
        finally:
            probe.close()

    def test_bind_failure_raises_without_starting_a_thread(self):
        clash = TelemetryReceiver(on_sample=lambda *_: None, port=self.receiver.port, host="127.0.0.1")
        with self.assertRaises(OSError):
            clash.start()
        clash.stop()


try:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt6.QtWidgets import QApplication
except ImportError:  # pragma: no cover - PyQt6 is optional for the tests above
    QApplication = None


@unittest.skipIf(QApplication is None, "PyQt6 not installed")
class LiveTelemetryControllerTests(unittest.TestCase):
    """Datagrams sent over loopback end up in the widget on the GUI thread."""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        from live_telemetry import LiveTelemetryController
        from widgets.telemetry import TelemetryWidget

        self.widget = TelemetryWidget()
        self.controller = LiveTelemetryController(self.widget, port=0, stale_timeout=0.3, host="127.0.0.1")
        self.assertTrue(self.controller.start())
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.sender.close()
        self.controller.stop()

    def process_until(self, condition, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.app.processEvents()
            if condition():
                return
            time.sleep(0.01)
        raise AssertionError("condition not met in time")

    def test_shows_no_data_then_live_then_stale(self):
        status = self.widget._status_label
        self.assertEqual(status.text(), "No telemetry received yet")
        self.assertEqual(self.widget._ball_labels[0].text(), "--")

        self.sender.sendto(FIRMWARE_PACKET, ("127.0.0.1", self.controller._receiver.port))
        self.process_until(lambda: status.text() == "Live telemetry")
        self.assertEqual(self.widget._ball_labels[0].text(), "80")
        self.assertEqual(self.widget._enabled_label.text(), "Disabled")

        self.process_until(lambda: status.text().startswith("Telemetry stale"))
        self.assertIn("italic", self.widget._ball_labels[0].styleSheet())

    def test_rejected_packets_update_counters_only(self):
        self.sender.sendto(b"garbage", ("127.0.0.1", self.controller._receiver.port))
        self.process_until(lambda: self.widget._rejected_label.text() == "1")
        self.assertIn("malformed JSON", self.widget._reject_reason_label.text())
        self.assertEqual(self.widget._status_label.text(), "No telemetry received yet")


if __name__ == "__main__":
    unittest.main()
