"""Scrolling time-series plots of the live telemetry stream.

The visible plot changes with the selected telemetry mode so the GUI can focus
on the active response instead of writing raw numbers beside the chart.
"""

from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QComboBox, QVBoxLayout, QWidget, QStackedWidget

X_COLOR = (80, 160, 255)
Y_COLOR = (255, 160, 60)
WINDOW_SECONDS = 10.0
TELEMETRY_MODES = {
    "Position": "position",
    "Velocity": "velocity",
    "Servo": "servo",
}


class PlotsWidget(QWidget):
    """Each telemetry mode shows its own plot page and legend set."""

    mode_changed = pyqtSignal(str)

    def __init__(self, window_seconds=WINDOW_SECONDS, parent=None):
        super().__init__(parent)
        self._window_seconds = window_seconds
        self._samples = deque()
        self._mode = "position"

        self._mode_combo = QComboBox()
        for label, mode in TELEMETRY_MODES.items():
            self._mode_combo.addItem(label, mode)
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)

        self._stack = None
        self._pages = {}
        self._build_pages()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._mode_combo)
        layout.addWidget(self._stack)

        self._set_mode("position")

    def _build_pages(self):
        self._stack = QStackedWidget()

        def solid(color):
            return pg.mkPen(color, width=2)

        def dashed(color):
            return pg.mkPen(color, width=2, style=Qt.PenStyle.DashLine)

        def setup_plot(plot, title, x_label, y_label, units):
            plot.setTitle(title)
            plot.setLabel("left", y_label, units=units)
            plot.setLabel("bottom", x_label, units="s")
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend(offset=(10, 10))
            plot.setYRange(-110, 110, padding=0)
            plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)

        # Position page
        position_page = QWidget()
        position_layout = QVBoxLayout(position_page)
        position_plot = pg.PlotWidget()
        error_plot = pg.PlotWidget()
        setup_plot(position_plot, "Position vs Setpoint", "Time", "Position", "mm")
        setup_plot(error_plot, "Tracking Error", "Time", "Error", "mm")
        error_plot.setXLink(position_plot)
        error_plot.addLine(y=0, pen=pg.mkPen((150, 150, 150), style=Qt.PenStyle.DotLine))
        position_layout.addWidget(position_plot)
        position_layout.addWidget(error_plot)
        self._pages["position"] = {
            "primary": position_plot,
            "secondary": error_plot,
            "curves": {
                "measured_x": position_plot.plot(pen=solid(X_COLOR), name="X measured", connect="finite"),
                "target_x": position_plot.plot(pen=dashed(X_COLOR), name="X target", connect="finite"),
                "measured_y": position_plot.plot(pen=solid(Y_COLOR), name="Y measured", connect="finite"),
                "target_y": position_plot.plot(pen=dashed(Y_COLOR), name="Y target", connect="finite"),
                "error_x": error_plot.plot(pen=solid(X_COLOR), name="X error", connect="finite"),
                "error_y": error_plot.plot(pen=solid(Y_COLOR), name="Y error", connect="finite"),
            },
        }
        self._stack.addWidget(position_page)

        # Velocity page
        velocity_page = QWidget()
        velocity_layout = QVBoxLayout(velocity_page)
        velocity_plot = pg.PlotWidget()
        setup_plot(velocity_plot, "Velocity", "Time", "Velocity", "mm/s")
        velocity_layout.addWidget(velocity_plot)
        self._pages["velocity"] = {
            "primary": velocity_plot,
            "secondary": None,
            "curves": {
                "velocity_x": velocity_plot.plot(pen=solid(X_COLOR), name="X velocity", connect="finite"),
                "velocity_y": velocity_plot.plot(pen=solid(Y_COLOR), name="Y velocity", connect="finite"),
            },
        }
        self._stack.addWidget(velocity_page)

        # Servo page
        servo_page = QWidget()
        servo_layout = QVBoxLayout(servo_page)
        servo_plot = pg.PlotWidget()
        error_plot = pg.PlotWidget()
        setup_plot(servo_plot, "Servo Output", "Time", "Servo", "us")
        setup_plot(error_plot, "Tracking Error", "Time", "Error", "mm")
        error_plot.setXLink(servo_plot)
        error_plot.addLine(y=0, pen=pg.mkPen((150, 150, 150), style=Qt.PenStyle.DotLine))
        servo_layout.addWidget(servo_plot)
        servo_layout.addWidget(error_plot)
        self._pages["servo"] = {
            "primary": servo_plot,
            "secondary": error_plot,
            "curves": {
                "servo_x": servo_plot.plot(pen=solid(X_COLOR), name="X servo", connect="finite"),
                "servo_y": servo_plot.plot(pen=solid(Y_COLOR), name="Y servo", connect="finite"),
                "servo_neutral_x": servo_plot.plot(pen=dashed(X_COLOR), name="X neutral", connect="finite"),
                "servo_neutral_y": servo_plot.plot(pen=dashed(Y_COLOR), name="Y neutral", connect="finite"),
                "error_x": error_plot.plot(pen=solid(X_COLOR), name="X error", connect="finite"),
                "error_y": error_plot.plot(pen=solid(Y_COLOR), name="Y error", connect="finite"),
            },
        }
        self._stack.addWidget(servo_page)

    def _on_mode_changed(self, index):
        mode = self._mode_combo.itemData(index)
        if mode is None:
            return
        self._set_mode(mode)
        self.mode_changed.emit(mode)

    def set_mode(self, mode):
        if mode not in TELEMETRY_MODES.values():
            return
        index = self._mode_combo.findData(mode)
        if index >= 0:
            self._mode_combo.setCurrentIndex(index)
        else:
            self._mode = mode
            self._set_mode(mode)

    def add_sample(self, time, measured=None, target=None, sample=None):
        """Append one sample. measured/target are (x, y) tuples or sample is a TelemetrySample."""
        if sample is not None:
            entry = {
                "time": float(time),
                "measured_x": getattr(sample, "ball_x_mm", None),
                "measured_y": getattr(sample, "ball_y_mm", None),
                "target_x": getattr(sample, "target_x_mm", None),
                "target_y": getattr(sample, "target_y_mm", None),
                "velocity_x": getattr(sample, "velocity_x_mm_s", None),
                "velocity_y": getattr(sample, "velocity_y_mm_s", None),
                "error_x": getattr(sample, "error_x_mm", None),
                "error_y": getattr(sample, "error_y_mm", None),
                "servo_x": getattr(sample, "servo_x_us", None),
                "servo_y": getattr(sample, "servo_y_us", None),
                "servo_neutral_x": getattr(sample, "servo_neutral_x_us", None),
                "servo_neutral_y": getattr(sample, "servo_neutral_y_us", None),
            }
        else:
            mx, my = measured if measured is not None else (np.nan, np.nan)
            tx, ty = target if target is not None else (np.nan, np.nan)
            entry = {
                "time": float(time),
                "measured_x": mx,
                "measured_y": my,
                "target_x": tx,
                "target_y": ty,
                "velocity_x": np.nan,
                "velocity_y": np.nan,
                "error_x": tx - mx,
                "error_y": ty - my,
                "servo_x": np.nan,
                "servo_y": np.nan,
                "servo_neutral_x": np.nan,
                "servo_neutral_y": np.nan,
            }

        self._samples.append(entry)
        while self._samples and self._samples[0]["time"] < time - self._window_seconds:
            self._samples.popleft()

        self._redraw()

    def clear(self):
        self._samples.clear()
        self._redraw()

    def _set_mode(self, mode):
        self._mode = mode
        if mode in self._pages:
            self._stack.setCurrentIndex(list(self._pages).index(mode))
        self._redraw()

    def _redraw(self):
        if not self._samples:
            for page in self._pages.values():
                for curve in page["curves"].values():
                    curve.setData([], [])
            return

        def series(key):
            values = []
            for sample in self._samples:
                value = sample.get(key, np.nan)
                values.append(np.nan if value is None else value)
            return np.array(values, dtype=float)

        time_values = np.array([sample["time"] for sample in self._samples], dtype=float)
        end = max(time_values[-1], self._window_seconds)

        for mode, page in self._pages.items():
            primary = page["primary"]
            secondary = page["secondary"]
            primary.setXRange(end - self._window_seconds, end, padding=0)
            if secondary is not None:
                secondary.setXRange(end - self._window_seconds, end, padding=0)

            if mode == "position":
                page["curves"]["measured_x"].setData(time_values, series("measured_x"))
                page["curves"]["measured_y"].setData(time_values, series("measured_y"))
                page["curves"]["target_x"].setData(time_values, series("target_x"))
                page["curves"]["target_y"].setData(time_values, series("target_y"))
                page["curves"]["error_x"].setData(time_values, series("error_x"))
                page["curves"]["error_y"].setData(time_values, series("error_y"))
            elif mode == "velocity":
                page["curves"]["velocity_x"].setData(time_values, series("velocity_x"))
                page["curves"]["velocity_y"].setData(time_values, series("velocity_y"))
            elif mode == "servo":
                page["curves"]["servo_x"].setData(time_values, series("servo_x"))
                page["curves"]["servo_y"].setData(time_values, series("servo_y"))
                page["curves"]["servo_neutral_x"].setData(time_values, series("servo_neutral_x"))
                page["curves"]["servo_neutral_y"].setData(time_values, series("servo_neutral_y"))
                page["curves"]["error_x"].setData(time_values, series("error_x"))
                page["curves"]["error_y"].setData(time_values, series("error_y"))

        if self._mode in self._pages:
            for name, curve in self._pages[self._mode]["curves"].items():
                curve.setVisible(True)
