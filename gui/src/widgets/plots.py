"""Scrolling time-series plots of the live telemetry stream.

The visible plot changes with the selected telemetry mode so the GUI can focus
on the active response instead of writing raw numbers beside the chart.
"""

from collections import deque
import math

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QComboBox, QGridLayout, QLabel, QHBoxLayout, QPushButton, QVBoxLayout, QWidget, QStackedWidget

import state

X_COLOR = (80, 160, 255)
Y_COLOR = (255, 160, 60)
WINDOW_SECONDS = 10.0
TELEMETRY_MODES = {
    "Position": "position",
    "Velocity": "velocity",
    "Servo": "servo",
    "PID Analysis": "pid_analysis",
    "Kalman": "kalman",
}


def _safe_float(value, default=0.0):
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return float(default)
    return numeric if math.isfinite(numeric) else float(default)


def _pid_gains_for_state(pid_state=None):
    pid_state = pid_state or state.app_state.pid
    return {
        "kp_x": _safe_float(getattr(pid_state, "kp_x", 0.0), 0.0),
        "ki_x": _safe_float(getattr(pid_state, "ki_x", 0.0), 0.0),
        "kd_x": _safe_float(getattr(pid_state, "kd_x", 0.0), 0.0),
        "kp_y": _safe_float(getattr(pid_state, "kp_y", 0.0), 0.0),
        "ki_y": _safe_float(getattr(pid_state, "ki_y", 0.0), 0.0),
        "kd_y": _safe_float(getattr(pid_state, "kd_y", 0.0), 0.0),
    }


def compute_pid_terms(samples, pid_state=None):
    """Return each sample enriched with P/I/D contributions for X and Y axes."""
    gains = _pid_gains_for_state(pid_state)
    samples = list(samples)
    integral_x = 0.0
    integral_y = 0.0
    previous_error_x = None
    previous_error_y = None
    previous_time = None
    result = []

    for sample in samples:
        time_value = _safe_float(sample.get("time", 0.0), 0.0)
        error_x = _safe_float(sample.get("error_x", 0.0), 0.0)
        error_y = _safe_float(sample.get("error_y", 0.0), 0.0)

        dt = 0.0 if previous_time is None else max(0.0, time_value - previous_time)
        if previous_time is not None and dt > 0.0:
            integral_x += (previous_error_x if previous_error_x is not None else 0.0) * dt
            integral_y += (previous_error_y if previous_error_y is not None else 0.0) * dt

        derivative_x = 0.0 if previous_error_x is None or dt <= 0 else (error_x - previous_error_x) / dt
        derivative_y = 0.0 if previous_error_y is None or dt <= 0 else (error_y - previous_error_y) / dt

        sample["p_x"] = gains["kp_x"] * error_x
        sample["i_x"] = gains["ki_x"] * integral_x
        sample["d_x"] = gains["kd_x"] * derivative_x
        sample["p_y"] = gains["kp_y"] * error_y
        sample["i_y"] = gains["ki_y"] * integral_y
        sample["d_y"] = gains["kd_y"] * derivative_y

        result.append(sample)

        previous_error_x = error_x
        previous_error_y = error_y
        previous_time = time_value

    return result


def compute_pid_contributions(samples, pid_state=None):
    """Aggregate the P/I/D percentages and steady-state error for each axis."""
    samples = compute_pid_terms(samples, pid_state)
    summary = {
        "x": {"p": 0.0, "i": 0.0, "d": 0.0, "steady_state_error": 0.0},
        "y": {"p": 0.0, "i": 0.0, "d": 0.0, "steady_state_error": 0.0},
    }
    if not samples:
        return summary

    for axis in ("x", "y"):
        totals = {name: 0.0 for name in ("p", "i", "d")}
        combined_total = 0.0
        for sample in samples:
            for term in ("p", "i", "d"):
                value = abs(_safe_float(sample.get(f"{term}_{axis}", 0.0), 0.0))
                totals[term] += value
                combined_total += value

        summary[axis]["p"] = (100.0 * totals["p"] / combined_total) if combined_total else 0.0
        summary[axis]["i"] = (100.0 * totals["i"] / combined_total) if combined_total else 0.0
        summary[axis]["d"] = (100.0 * totals["d"] / combined_total) if combined_total else 0.0

        errors = [abs(_safe_float(sample.get(f"error_{axis}", 0.0), 0.0)) for sample in samples]
        if errors:
            tail_count = max(1, int(math.ceil(len(errors) * 0.25)))
            tail_errors = errors[-tail_count:]
            summary[axis]["steady_state_error"] = sum(tail_errors) / len(tail_errors)

    return summary


class PlotsWidget(QWidget):
    """Each telemetry mode shows its own plot page and legend set."""

    mode_changed = pyqtSignal(str)

    def __init__(self, window_seconds=WINDOW_SECONDS, parent=None):
        super().__init__(parent)
        self._window_seconds = window_seconds
        self._samples = deque()
        self._mode = "position"
        self._paused = False

        self._mode_combo = QComboBox()
        for label, mode in TELEMETRY_MODES.items():
            self._mode_combo.addItem(label, mode)
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)

        self._clear_button = QPushButton("Clear plots")
        self._clear_button.clicked.connect(self.clear)
        self._pause_button = QPushButton("Pause")
        self._pause_button.clicked.connect(self._toggle_pause)

        self._selection_label = QLabel("Selected: none")
        self._selection_label.setWordWrap(True)

        self._stack = None
        self._pages = {}
        self._build_pages()

        top_layout = QHBoxLayout()
        top_layout.addWidget(self._mode_combo)
        top_layout.addWidget(self._clear_button)
        top_layout.addWidget(self._pause_button)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(top_layout)
        layout.addWidget(self._selection_label)
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
            plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)
            plot.setYRange(-110, 110, padding=0)
            plot.setMenuEnabled(False)
            plot.setMouseEnabled(x=True, y=True)
            plot.setMouseTracking(True)
            plot.scene().sigMouseClicked.connect(lambda event, plot=plot: self._on_plot_click(plot, event))

        def maybe_add_selection_y_line(plot):
            plot.addLine(y=0, pen=pg.mkPen((150, 150, 150), style=Qt.PenStyle.DotLine))

        # Position page
        position_page = QWidget()
        position_layout = QVBoxLayout(position_page)
        position_plot = pg.PlotWidget()
        error_plot = pg.PlotWidget()
        setup_plot(position_plot, "Position vs Setpoint", "Time", "Position", "mm")
        setup_plot(error_plot, "Tracking Error", "Time", "Error", "mm")
        error_plot.setXLink(position_plot)
        maybe_add_selection_y_line(error_plot)
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
        maybe_add_selection_y_line(error_plot)
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

        # PID analysis page
        pid_page = QWidget()
        pid_layout = QVBoxLayout(pid_page)
        pid_plot = pg.PlotWidget()
        setup_plot(pid_plot, "PID contribution breakdown", "Time", "Contribution", "mm")
        summary = QWidget()
        summary_layout = QGridLayout(summary)
        self._pid_summary_labels = {
            "x_steady": QLabel("X steady-state error: -- mm"),
            "x_p": QLabel("X P: -- %"),
            "x_i": QLabel("X I: -- %"),
            "x_d": QLabel("X D: -- %"),
            "y_steady": QLabel("Y steady-state error: -- mm"),
            "y_p": QLabel("Y P: -- %"),
            "y_i": QLabel("Y I: -- %"),
            "y_d": QLabel("Y D: -- %"),
        }
        for row, label in enumerate(("x_steady", "x_p", "x_i", "x_d", "y_steady", "y_p", "y_i", "y_d")):
            summary_layout.addWidget(self._pid_summary_labels[label], row, 0)
        pid_layout.addWidget(pid_plot)
        pid_layout.addWidget(summary)
        self._pages["pid_analysis"] = {
            "primary": pid_plot,
            "secondary": None,
            "curves": {
                "p_x": pid_plot.plot(pen=solid(X_COLOR), name="X P", connect="finite"),
                "i_x": pid_plot.plot(pen=dashed(X_COLOR), name="X I", connect="finite"),
                "d_x": pid_plot.plot(pen=solid((130, 130, 255)), name="X D", connect="finite"),
                "p_y": pid_plot.plot(pen=solid(Y_COLOR), name="Y P", connect="finite"),
                "i_y": pid_plot.plot(pen=dashed(Y_COLOR), name="Y I", connect="finite"),
                "d_y": pid_plot.plot(pen=solid((255, 180, 80)), name="Y D", connect="finite"),
            },
            "summary_labels": self._pid_summary_labels,
        }
        self._stack.addWidget(pid_page)

        # Kalman analysis page
        kalman_page = QWidget()
        kalman_layout = QVBoxLayout(kalman_page)
        estimate_plot = pg.PlotWidget()
        innovation_plot = pg.PlotWidget()
        setup_plot(estimate_plot, "Kalman estimate vs measurement", "Time", "Position", "mm")
        setup_plot(innovation_plot, "Innovation", "Time", "Residual", "mm")
        innovation_plot.setXLink(estimate_plot)
        maybe_add_selection_y_line(innovation_plot)
        kalman_layout.addWidget(estimate_plot)
        kalman_layout.addWidget(innovation_plot)
        self._pages["kalman"] = {
            "primary": estimate_plot,
            "secondary": innovation_plot,
            "curves": {
                "measurement_x": estimate_plot.plot(pen=solid(X_COLOR), name="X measured", connect="finite"),
                "filtered_x": estimate_plot.plot(pen=dashed(X_COLOR), name="X filtered", connect="finite"),
                "measurement_y": estimate_plot.plot(pen=solid(Y_COLOR), name="Y measured", connect="finite"),
                "filtered_y": estimate_plot.plot(pen=dashed(Y_COLOR), name="Y filtered", connect="finite"),
                "innovation_x": innovation_plot.plot(pen=solid(X_COLOR), name="X innovation", connect="finite"),
                "innovation_y": innovation_plot.plot(pen=solid(Y_COLOR), name="Y innovation", connect="finite"),
            },
        }
        self._stack.addWidget(kalman_page)

    def _on_plot_click(self, plot, event):
        if event.button() != Qt.MouseButton.LeftButton or not self._samples:
            return

        pos = plot.getPlotItem().vb.mapSceneToView(event.scenePos())
        clicked_time = float(pos.x())
        nearest = None
        nearest_delta = None
        for sample in self._samples:
            delta = abs(_safe_float(sample.get("time", 0.0), 0.0) - clicked_time)
            if nearest_delta is None or delta < nearest_delta:
                nearest = sample
                nearest_delta = delta

        if nearest is None or nearest_delta is None:
            return

        tolerance = max(0.15, self._window_seconds * 0.1)
        if nearest_delta > tolerance:
            return

        self._selection_label.setText(self._describe_sample(nearest))

    def _describe_sample(self, sample):
        time_value = _safe_float(sample.get("time", 0.0), 0.0)
        if self._mode == "position":
            measured_x = _safe_float(sample.get("measured_x", np.nan), np.nan)
            measured_y = _safe_float(sample.get("measured_y", np.nan), np.nan)
            target_x = _safe_float(sample.get("target_x", np.nan), np.nan)
            target_y = _safe_float(sample.get("target_y", np.nan), np.nan)
            return (
                f"Selected: t={time_value:.3f}s | measured=({measured_x:.2f}, {measured_y:.2f}) mm | "
                f"target=({target_x:.2f}, {target_y:.2f}) mm"
            )
        if self._mode == "velocity":
            velocity_x = _safe_float(sample.get("velocity_x", np.nan), np.nan)
            velocity_y = _safe_float(sample.get("velocity_y", np.nan), np.nan)
            return f"Selected: t={time_value:.3f}s | velocity=({velocity_x:.2f}, {velocity_y:.2f}) mm/s"
        if self._mode == "servo":
            servo_x = _safe_float(sample.get("servo_x", np.nan), np.nan)
            servo_y = _safe_float(sample.get("servo_y", np.nan), np.nan)
            error_x = _safe_float(sample.get("error_x", np.nan), np.nan)
            error_y = _safe_float(sample.get("error_y", np.nan), np.nan)
            return (
                f"Selected: t={time_value:.3f}s | servo=({servo_x:.0f}, {servo_y:.0f}) us | "
                f"error=({error_x:.2f}, {error_y:.2f}) mm"
            )
        if self._mode == "pid_analysis":
            return (
                f"Selected: t={time_value:.3f}s | P=({sample.get('p_x', 0.0):.2f}, {sample.get('p_y', 0.0):.2f}) | "
                f"I=({sample.get('i_x', 0.0):.2f}, {sample.get('i_y', 0.0):.2f}) | "
                f"D=({sample.get('d_x', 0.0):.2f}, {sample.get('d_y', 0.0):.2f})"
            )
        if self._mode == "kalman":
            measured_x = _safe_float(sample.get("measured_x", np.nan), np.nan)
            measured_y = _safe_float(sample.get("measured_y", np.nan), np.nan)
            filtered_x = _safe_float(sample.get("filtered_x", np.nan), np.nan)
            filtered_y = _safe_float(sample.get("filtered_y", np.nan), np.nan)
            innovation_x = _safe_float(sample.get("innovation_x", np.nan), np.nan)
            innovation_y = _safe_float(sample.get("innovation_y", np.nan), np.nan)
            return (
                f"Selected: t={time_value:.3f}s | measured=({measured_x:.2f}, {measured_y:.2f}) mm | "
                f"filtered=({filtered_x:.2f}, {filtered_y:.2f}) mm | innovation=({innovation_x:.2f}, {innovation_y:.2f}) mm"
            )
        return f"Selected: t={time_value:.3f}s"

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

    def _toggle_pause(self):
        self._paused = not self._paused
        self._pause_button.setText("Resume" if self._paused else "Pause")
        if not self._paused:
            self._redraw()

    def _update_pid_summary(self):
        page = self._pages.get("pid_analysis")
        if page is None or "summary_labels" not in page:
            return

        if not self._samples:
            for label in page["summary_labels"].values():
                label.setText(label.text().split(":", 1)[0] + ": --")
            return

        summary = compute_pid_contributions(self._samples, state.app_state.pid)
        labels = page["summary_labels"]
        labels["x_steady"].setText(f"X steady-state error: {summary['x']['steady_state_error']:.2f} mm")
        labels["x_p"].setText(f"X P: {summary['x']['p']:.1f}%")
        labels["x_i"].setText(f"X I: {summary['x']['i']:.1f}%")
        labels["x_d"].setText(f"X D: {summary['x']['d']:.1f}%")
        labels["y_steady"].setText(f"Y steady-state error: {summary['y']['steady_state_error']:.2f} mm")
        labels["y_p"].setText(f"Y P: {summary['y']['p']:.1f}%")
        labels["y_i"].setText(f"Y I: {summary['y']['i']:.1f}%")
        labels["y_d"].setText(f"Y D: {summary['y']['d']:.1f}%")

    def add_sample(self, time, measured=None, target=None, sample=None):
        """Append one sample. measured/target are (x, y) tuples or sample is a TelemetrySample."""
        if sample is not None:
            entry = {
                "time": float(time),
                "measured_x": _safe_float(getattr(sample, "measurement_x_mm", getattr(sample, "ball_x_mm", None)), np.nan),
                "measured_y": _safe_float(getattr(sample, "measurement_y_mm", getattr(sample, "ball_y_mm", None)), np.nan),
                "filtered_x": _safe_float(getattr(sample, "ball_x_mm", None), np.nan),
                "filtered_y": _safe_float(getattr(sample, "ball_y_mm", None), np.nan),
                "target_x": _safe_float(getattr(sample, "target_x_mm", None), np.nan),
                "target_y": _safe_float(getattr(sample, "target_y_mm", None), np.nan),
                "velocity_x": _safe_float(getattr(sample, "velocity_x_mm_s", None), np.nan),
                "velocity_y": _safe_float(getattr(sample, "velocity_y_mm_s", None), np.nan),
                "error_x": _safe_float(getattr(sample, "error_x_mm", None), np.nan),
                "error_y": _safe_float(getattr(sample, "error_y_mm", None), np.nan),
                "servo_x": _safe_float(getattr(sample, "servo_x_us", None), np.nan),
                "servo_y": _safe_float(getattr(sample, "servo_y_us", None), np.nan),
                "servo_neutral_x": _safe_float(getattr(sample, "servo_neutral_x_us", None), np.nan),
                "servo_neutral_y": _safe_float(getattr(sample, "servo_neutral_y_us", None), np.nan),
                "innovation_x": _safe_float(getattr(sample, "ball_x_mm", None), np.nan) - _safe_float(getattr(sample, "measurement_x_mm", getattr(sample, "ball_x_mm", None)), np.nan),
                "innovation_y": _safe_float(getattr(sample, "ball_y_mm", None), np.nan) - _safe_float(getattr(sample, "measurement_y_mm", getattr(sample, "ball_y_mm", None)), np.nan),
            }
        else:
            mx, my = measured if measured is not None else (np.nan, np.nan)
            tx, ty = target if target is not None else (np.nan, np.nan)
            entry = {
                "time": float(time),
                "measured_x": _safe_float(mx, np.nan),
                "measured_y": _safe_float(my, np.nan),
                "filtered_x": _safe_float(mx, np.nan),
                "filtered_y": _safe_float(my, np.nan),
                "target_x": _safe_float(tx, np.nan),
                "target_y": _safe_float(ty, np.nan),
                "velocity_x": np.nan,
                "velocity_y": np.nan,
                "error_x": _safe_float(tx, 0.0) - _safe_float(mx, 0.0),
                "error_y": _safe_float(ty, 0.0) - _safe_float(my, 0.0),
                "servo_x": np.nan,
                "servo_y": np.nan,
                "servo_neutral_x": np.nan,
                "servo_neutral_y": np.nan,
                "innovation_x": np.nan,
                "innovation_y": np.nan,
            }

        self._samples.append(entry)
        compute_pid_terms(self._samples, state.app_state.pid)
        self._redraw()

    def clear(self):
        self._samples.clear()
        self._selection_label.setText("Selected: none")
        self._redraw()

    def _set_mode(self, mode):
        self._mode = mode
        if mode in self._pages:
            self._stack.setCurrentIndex(list(self._pages).index(mode))
        self._selection_label.setText("Selected: none")
        self._redraw()

    def _redraw(self):
        if not self._samples:
            for page in self._pages.values():
                for curve in page["curves"].values():
                    curve.setData([], [])
            self._update_pid_summary()
            return

        compute_pid_terms(self._samples, state.app_state.pid)
        self._update_pid_summary()

        def series(key):
            values = []
            for sample in self._samples:
                value = sample.get(key, np.nan)
                values.append(np.nan if value is None else _safe_float(value, np.nan))
            return np.array(values, dtype=float)

        time_values = np.array([_safe_float(sample["time"], 0.0) for sample in self._samples], dtype=float)
        end = max(time_values[-1], self._window_seconds)

        for mode, page in self._pages.items():
            primary = page["primary"]
            secondary = page["secondary"]
            if not self._paused:
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
            elif mode == "pid_analysis":
                page["curves"]["p_x"].setData(time_values, series("p_x"))
                page["curves"]["i_x"].setData(time_values, series("i_x"))
                page["curves"]["d_x"].setData(time_values, series("d_x"))
                page["curves"]["p_y"].setData(time_values, series("p_y"))
                page["curves"]["i_y"].setData(time_values, series("i_y"))
                page["curves"]["d_y"].setData(time_values, series("d_y"))
            elif mode == "kalman":
                page["curves"]["measurement_x"].setData(time_values, series("measured_x"))
                page["curves"]["measurement_y"].setData(time_values, series("measured_y"))
                page["curves"]["filtered_x"].setData(time_values, series("filtered_x"))
                page["curves"]["filtered_y"].setData(time_values, series("filtered_y"))
                page["curves"]["innovation_x"].setData(time_values, series("innovation_x"))
                page["curves"]["innovation_y"].setData(time_values, series("innovation_y"))

        if self._mode in self._pages:
            for curve in self._pages[self._mode]["curves"].values():
                curve.setVisible(True)
