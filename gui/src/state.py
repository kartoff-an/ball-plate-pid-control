from dataclasses import dataclass, field
from threading import Lock
from typing import Optional
import config

@dataclass
class TargetState:
    x: float = config.DEFAULT_TARGET_X
    y: float = config.DEFAULT_TARGET_Y

@dataclass
class BallState:
    detected: bool = False
    x: Optional[float] = None
    y: Optional[float] = None
    measurement_age_ms: Optional[int] = None
    sequence: Optional[int] = None

@dataclass
class PIDState:
    error_x: Optional[float] = None
    error_y: Optional[float] = None
    output_x_us: Optional[int] = None
    output_y_us: Optional[int] = None
    kp_x: float = 0.0
    ki_x: float = 0.0
    kd_x: float = 0.0
    kp_y: float = 0.0
    ki_y: float = 0.0
    kd_y: float = 0.0

@dataclass
class ConnectionState:
    connected: bool = False
    last_error: Optional[str] = None

@dataclass
class AppState:
    target: TargetState = field(default_factory=TargetState)
    ball: BallState = field(default_factory=BallState)
    pid: PIDState = field(default_factory=PIDState)
    connection: ConnectionState = field(default_factory=ConnectionState)
    controller_enabled: bool = False

    _lock: Lock = field(default_factory=Lock, repr=False, compare=False)

app_state = AppState()

print(TargetState.x)