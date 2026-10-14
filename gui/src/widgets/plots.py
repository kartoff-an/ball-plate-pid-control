"""Scrolling time-series plots of ball position versus setpoint and tracking error.

Samples are added one at a time with add_sample(). A position of None leaves
a gap in the plot instead of drawing a misleading line to zero.
"""

from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QVBoxLayout, QWidget

X_COLOR = (80, 160, 255)
Y_COLOR = (255, 160, 60)
WINDOW_SECONDS = 10.0


class PlotsWidget(QWidget):
    """Positions and errors are plate coordinates in mm; time is in seconds.

    Error is target minus measured, matching the telemetry panel.
    """

    def __init__(self, window_seconds=WINDOW_SECONDS, parent=None):
        super().__init__(parent)
        self._window_seconds = window_seconds
        # time, measured x, measured y, target x, target y
        self._samples = deque()

        self._position_plot = pg.PlotWidget()
        self._position_plot.setTitle("Position vs Setpoint")
        self._position_plot.setLabel("left", "Position", units="mm")
        self._position_plot.setLabel("bottom", "Time", units="s")

        self._error_plot = pg.PlotWidget()
        self._error_plot.setTitle("Tracking Error")
        self._error_plot.setLabel("left", "Error", units="mm")
        self._error_plot.setLabel("bottom", "Time", units="s")
        self._error_plot.setXLink(self._position_plot)

        for plot in (self._position_plot, self._error_plot):
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend(offset=(10, 10))

        def solid(color):
            return pg.mkPen(color, width=2)

        def dashed(color):
            return pg.mkPen(color, width=2, style=Qt.PenStyle.DashLine)

        # connect="finite" breaks the line at NaN, which is how gaps are drawn.
        self._curves = {
            "measured_x": self._position_plot.plot(pen=solid(X_COLOR), name="X measured", connect="finite"),
            "target_x": self._position_plot.plot(pen=dashed(X_COLOR), name="X target", connect="finite"),
            "measured_y": self._position_plot.plot(pen=solid(Y_COLOR), name="Y measured", connect="finite"),
            "target_y": self._position_plot.plot(pen=dashed(Y_COLOR), name="Y target", connect="finite"),
            "error_x": self._error_plot.plot(pen=solid(X_COLOR), name="X error", connect="finite"),
            "error_y": self._error_plot.plot(pen=solid(Y_COLOR), name="Y error", connect="finite"),
        }
        self._error_plot.addLine(y=0, pen=pg.mkPen((150, 150, 150), style=Qt.PenStyle.DotLine))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._position_plot)
        layout.addWidget(self._error_plot)

    def add_sample(self, time, measured, target):
        """Append one sample. measured and target are (x, y) tuples or None."""
        mx, my = measured if measured is not None else (np.nan, np.nan)
        tx, ty = target if target is not None else (np.nan, np.nan)
        self._samples.append((time, mx, my, tx, ty))

        while self._samples and self._samples[0][0] < time - self._window_seconds:
            self._samples.popleft()

        self._redraw()

    def clear(self):
        self._samples.clear()
        self._redraw()

    def _redraw(self):
        if not self._samples:
            for curve in self._curves.values():
                curve.setData([], [])
            return

        # NaN in either input makes the error NaN, so gaps carry through.
        t, mx, my, tx, ty = np.array(self._samples, dtype=float).T
        self._curves["measured_x"].setData(t, mx)
        self._curves["measured_y"].setData(t, my)
        self._curves["target_x"].setData(t, tx)
        self._curves["target_y"].setData(t, ty)
        self._curves["error_x"].setData(t, tx - mx)
        self._curves["error_y"].setData(t, ty - my)

        end = max(t[-1], self._window_seconds)
        self._position_plot.setXRange(end - self._window_seconds, end, padding=0)


if __name__ == "__main__":
    # Standalone demo with mock data. From gui/: python -m widgets.plots
    import sys
    import time

    from PyQt6.QtCore import QTimer
    from PyQt6.QtWidgets import QApplication

    from mock_data import MockCamera, pixels_to_mm

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    camera = MockCamera()
    plots = PlotsWidget()
    plots.setWindowTitle("Plots (mock data)")
    plots.resize(800, 600)

    target = (0.0, 0.0)
    start = time.perf_counter()

    def tick():
        t = time.perf_counter() - start
        plots.add_sample(t, pixels_to_mm(camera.ball_position(t)), target)

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(33)

    plots.show()
    sys.exit(app.exec())
