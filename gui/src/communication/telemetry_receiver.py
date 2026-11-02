"""Receives live telemetry broadcast by the ESP32-S3 over UDP.

The firmware (firmware/esp32_s3/main/telemetry.c) broadcasts one JSON object
per datagram to 192.168.4.255 on config.PC_TELEMETRY_PORT while it has a fresh
ball measurement. It
sends nothing when the ball is missing, so "no data" is detected from the time
since the last packet, not from the packet contents.

Nothing in this module uses Qt. The receiver runs its socket on a background
thread and hands results to callbacks; the GUI decides how to get them onto
its own thread.
"""

import json
import logging
import math
import socket
import threading
import time
from dataclasses import dataclass
from enum import Enum

import config

# The firmware's send buffer is 512 bytes, so anything larger is not ours.
MAX_DATAGRAM_BYTES = 2048
UINT32_MAX = 2**32 - 1

# How long recvfrom() blocks before re-checking whether to stop.
_POLL_INTERVAL_S = 0.25
_LOG_INTERVAL_S = 5.0

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class TelemetrySample:
    """One validated telemetry packet.

    Errors are plate millimeters (target minus measured). Outputs and servo
    commands are microseconds. received_at is time.monotonic() on this
    computer, since the payload carries no timestamp.
    """

    mode: str
    sequence: int
    timestamp_us: int
    controller_enabled: bool
    received_at: float

    ball_detected: bool = None
    measurement_x_mm: float = None
    measurement_y_mm: float = None
    ball_x_mm: float = None
    ball_y_mm: float = None
    target_x_mm: float = None
    target_y_mm: float = None
    error_x_mm: float = None
    error_y_mm: float = None
    measurement_age_ms: int = None

    velocity_x_mm_s: float = None
    velocity_y_mm_s: float = None

    pid_output_x_us: float = None
    pid_output_y_us: float = None
    output_x_us: float = None
    output_y_us: float = None

    servo_neutral_x_us: int = None
    servo_neutral_y_us: int = None
    servo_x_us: int = None
    servo_y_us: int = None

    servo_saturated_x: bool = None
    servo_saturated_y: bool = None


class TelemetryParseError(ValueError):
    """A datagram that is not a valid telemetry packet."""


_BOOL = "boolean"
_INT = "integer"
_NUMBER = "number"

_COMMON_FIELDS = {
    "sequence": _INT,
    "timestamp_us": _INT,
    "controller_enabled": _BOOL,
}

_MODE_FIELDS = {
    "position": {
        "ball_detected": _BOOL,
        "measurement_x_mm": _NUMBER,
        "measurement_y_mm": _NUMBER,
        "ball_x_mm": _NUMBER,
        "ball_y_mm": _NUMBER,
        "target_x_mm": _NUMBER,
        "target_y_mm": _NUMBER,
        "error_x_mm": _NUMBER,
        "error_y_mm": _NUMBER,
        "measurement_age_ms": _INT,
    },
    "velocity": {
        "ball_detected": _BOOL,
        "measurement_x_mm": _NUMBER,
        "measurement_y_mm": _NUMBER,
        "ball_x_mm": _NUMBER,
        "ball_y_mm": _NUMBER,
        "target_x_mm": _NUMBER,
        "target_y_mm": _NUMBER,
        "velocity_x_mm_s": _NUMBER,
        "velocity_y_mm_s": _NUMBER,
        "measurement_age_ms": _INT,
    },
    "servo": {
        "error_x_mm": _NUMBER,
        "error_y_mm": _NUMBER,
        "pid_output_x_us": _NUMBER,
        "pid_output_y_us": _NUMBER,
        "output_x_us": _NUMBER,
        "output_y_us": _NUMBER,
        "servo_neutral_x_us": _INT,
        "servo_neutral_y_us": _INT,
        "servo_x_us": _INT,
        "servo_y_us": _INT,
        "servo_saturated_x": _BOOL,
        "servo_saturated_y": _BOOL,
    },
}

_NON_NEGATIVE = {
    "sequence",
    "timestamp_us",
    "measurement_age_ms",
    "servo_neutral_x_us",
    "servo_neutral_y_us",
    "servo_x_us",
    "servo_y_us",
}


def _reject_constant(name):
    raise TelemetryParseError(f"non-finite number {name} is not valid JSON")


def parse_telemetry(datagram, received_at):
    """Decode one UDP datagram into a TelemetrySample or raise TelemetryParseError.

    Unknown extra fields are ignored so a newer firmware does not break the GUI.
    """
    if len(datagram) > MAX_DATAGRAM_BYTES:
        raise TelemetryParseError(f"datagram too large ({len(datagram)} bytes)")

    try:
        text = datagram.decode("utf-8")
    except UnicodeDecodeError as error:
        raise TelemetryParseError(f"not UTF-8: {error}") from None

    try:
        payload = json.loads(text, parse_constant=_reject_constant)
    except json.JSONDecodeError as error:
        raise TelemetryParseError(f"malformed JSON: {error.msg}") from None

    if not isinstance(payload, dict):
        raise TelemetryParseError("payload is not a JSON object")

    packet_type = payload.get("type")
    if packet_type != "telemetry":
        raise TelemetryParseError(f"unrelated packet type {packet_type!r}")
    
    mode = payload.get("mode")
    if not isinstance(mode, str) or mode not in _MODE_FIELDS:
        raise TelemetryParseError(f"unsupported telemetry mode {mode!r}")

    expected_fields = {**_COMMON_FIELDS, **_MODE_FIELDS[mode]}

    missing = [name for name in expected_fields if name not in payload]
    if missing:
        raise TelemetryParseError(f"missing fields: {', '.join(missing)}")

    values = {name: _check_field(name, payload[name], kind) for name, kind in expected_fields.items()}

    if values["sequence"] > UINT32_MAX:
        raise TelemetryParseError("sequence exceeds 32 bits")

    return TelemetrySample(
        mode=mode,
        received_at=received_at,
        **values,
    )


def _check_field(name, value, kind):
    # bool is a subclass of int in Python, so it has to be excluded explicitly.
    if kind == _BOOL:
        ok = isinstance(value, bool)
    elif kind == _INT:
        ok = isinstance(value, int) and not isinstance(value, bool)
    else:
        ok = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    if not ok:
        raise TelemetryParseError(f"{name} must be a {kind}, got {value!r}")
    if name in _NON_NEGATIVE and value < 0:
        raise TelemetryParseError(f"{name} must not be negative, got {value}")

    return float(value) if kind == _NUMBER else value


class SequenceTracker:
    """Counts packets lost between consecutive sequence numbers.

    The firmware counter is uint32 and wraps. A large backwards jump is read as
    a firmware restart rather than four billion lost packets.
    """

    def __init__(self):
        self._last = None
        self.missed = 0
        self.restarts = 0

    def update(self, sequence):
        """Record a sequence number and return how many packets were skipped."""
        last, self._last = self._last, sequence
        if last is None:
            return 0

        gap = (sequence - last - 1) % (UINT32_MAX + 1)
        if gap >= 2**31:
            # Duplicate, reordered, or restarted counter: not a loss.
            if sequence < last:
                self.restarts += 1
            return 0

        self.missed += gap
        return gap


class LinkState(Enum):
    NO_DATA = "no data"
    LIVE = "live"
    STALE = "stale"


class LinkMonitor:
    """Tracks data freshness from local receive times."""

    def __init__(self, stale_timeout=config.TELEMETRY_STALE_TIMEOUT_S):
        self.stale_timeout = stale_timeout
        self.last_received = None

    def packet_received(self, now):
        self.last_received = now

    def state(self, now):
        if self.last_received is None:
            return LinkState.NO_DATA
        if now - self.last_received > self.stale_timeout:
            return LinkState.STALE
        return LinkState.LIVE

    def seconds_since_last(self, now):
        return None if self.last_received is None else now - self.last_received


@dataclass(frozen=True)
class ReceiverStats:
    received: int = 0
    rejected: int = 0
    missed: int = 0
    restarts: int = 0


class _RateLimitedLog:
    """Logs the first message, then at most one every interval with a skip count."""

    def __init__(self, interval=_LOG_INTERVAL_S):
        self._interval = interval
        self._next = 0.0
        self._suppressed = 0

    def warning(self, message, now):
        if now < self._next:
            self._suppressed += 1
            return
        if self._suppressed:
            message = f"{message} ({self._suppressed} similar messages suppressed)"
        log.warning(message)
        self._suppressed = 0
        self._next = now + self._interval


class TelemetryReceiver:
    """Listens for telemetry datagrams on a background thread.

    on_sample(sample, stats) is called for each valid packet and
    on_rejected(reason, stats) for each invalid one, both from the receiver
    thread. on_failed(message) is called if the socket stops working.
    """

    def __init__(self, on_sample, on_rejected=None, on_failed=None, port=config.PC_TELEMETRY_PORT, host=config.PC_TELEMETRY_HOST):
        self._on_sample = on_sample
        self._on_rejected = on_rejected
        self._on_failed = on_failed
        self._address = (host, port)
        self._socket = None
        self._thread = None
        self._stop = threading.Event()
        self._sequence = SequenceTracker()
        self._received = 0
        self._rejected = 0
        self._reject_log = _RateLimitedLog()

    @property
    def port(self):
        """The bound port, useful when constructed with port=0."""
        return self._socket.getsockname()[1] if self._socket else None

    def start(self):
        """Bind the socket and start listening. Raises OSError if the port is unavailable."""
        if self._thread is not None:
            raise RuntimeError("receiver already started")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(self._address)
            sock.settimeout(_POLL_INTERVAL_S)
        except OSError:
            sock.close()
            raise

        self._socket = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="telemetry-receiver", daemon=True)
        self._thread.start()
        log.info("Listening for telemetry on UDP port %d", self.port)

    def stop(self, timeout=2.0):
        """Stop the thread and release the socket. Safe to call more than once."""
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout)
            if thread.is_alive():
                log.warning("Telemetry receiver thread did not stop within %.1f s", timeout)

        # The thread closes the socket on exit; this covers a thread that hung.
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def _stats(self):
        return ReceiverStats(
            received=self._received,
            rejected=self._rejected,
            missed=self._sequence.missed,
            restarts=self._sequence.restarts,
        )

    def _run(self):
        sock = self._socket
        try:
            while not self._stop.is_set():
                try:
                    datagram, sender = sock.recvfrom(MAX_DATAGRAM_BYTES + 1)
                except socket.timeout:
                    continue
                except ConnectionResetError:
                    # Windows reports ICMP errors on UDP sockets this way; harmless here.
                    continue
                except OSError as error:
                    if not self._stop.is_set():
                        log.error("Telemetry socket failed: %s", error)
                        if self._on_failed:
                            self._on_failed(str(error))
                    return

                self._handle(datagram, sender, time.monotonic())
        finally:
            sock.close()

    def _handle(self, datagram, sender, now):
        try:
            sample = parse_telemetry(datagram, received_at=now)
        except TelemetryParseError as error:
            self._rejected += 1
            reason = str(error)
            self._reject_log.warning(f"Ignored datagram from {sender[0]}:{sender[1]}: {reason}", now)
            if self._on_rejected:
                self._on_rejected(reason, self._stats())
            return

        self._received += 1
        self._sequence.update(sample.sequence)
        self._on_sample(sample, self._stats())
