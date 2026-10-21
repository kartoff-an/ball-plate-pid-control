__version__= "0.1.0"

from main_window import BallPlateWindow

from widgets.camera_preview import CameraPreviewWidget
from widgets.plots import PlotsWidget
from widgets.telemetry import TelemetryWidget

__all__ = [
    "BallPlateWindow",
    "CameraPreviewWidget",
    "PlotsWidget",
    "TelemetryWidget"
]