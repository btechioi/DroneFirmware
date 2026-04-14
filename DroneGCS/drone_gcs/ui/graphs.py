"""Real-time graphs for PID tuning visualization"""

from __future__ import annotations

import logging
import numpy as np
from collections import deque
from typing import Optional, Callable
from dataclasses import dataclass

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QGroupBox,
    QCheckBox,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QColor

try:
    import pyqtgraph as pg

    PyqtgraphAvailable = True
except ImportError:
    PyqtgraphAvailable = False
    pg = None

logger = logging.getLogger(__name__)


@dataclass
class PIDTuneData:
    timestamp: float
    setpoint: float
    measurement: float
    error: float
    kp: float
    ki: float
    kd: float
    output: float
    axis: int = 0


class RollingBuffer:
    def __init__(self, max_size: int = 1000):
        self.max_size = max_size
        self.data = deque(maxlen=max_size)

    def append(self, value: float) -> None:
        self.data.append(value)

    def get_array(self) -> np.ndarray:
        return np.array(list(self.data))

    def clear(self) -> None:
        self.data.clear()

    def __len__(self) -> int:
        return len(self.data)


class PIDGraphWidget(QWidget):
    data_received = pyqtSignal(PIDTuneData)

    def __init__(
        self,
        title: str = "PID Response",
        max_points: int = 500,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.max_points = max_points
        self.title = title

        self._init_ui()
        self._init_buffers()

    def _init_ui(self) -> None:
        if not PyqtgraphAvailable:
            layout = QVBoxLayout(self)
            layout.addWidget(QLabel("pyqtgraph not available"))
            return

        layout = QVBoxLayout(self)

        header = QHBoxLayout()
        header.addWidget(QLabel(self.title))
        self.clear_btn = QPushButton("Clear")
        self.clear_btn.clicked.connect(self.clear)
        header.addWidget(self.clear_btn)
        layout.addLayout(header)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("#1a1a2e")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("left", "Value", units="deg/s")
        self.plot_widget.setLabel("bottom", "Time", units="samples")
        self.plot_widget.addLegend(offset=(10, 10))

        self.plot_widget.setYRange(-180, 180)

        layout.addWidget(self.plot_widget)

        self.setpoint_curve = self.plot_widget.plot(
            pen=pg.mkPen(color="#00ff00", width=2), name="Setpoint"
        )
        self.measurement_curve = self.plot_widget.plot(
            pen=pg.mkPen(color="#ff6600", width=2), name="Measurement"
        )
        self.error_curve = self.plot_widget.plot(
            pen=pg.mkPen(color="#ff0066", width=1, style=Qt.PenStyle.DashLine),
            name="Error",
        )

    def _init_buffers(self) -> None:
        self.setpoint_buffer = RollingBuffer(self.max_points)
        self.measurement_buffer = RollingBuffer(self.max_points)
        self.error_buffer = RollingBuffer(self.max_points)

    def add_data(self, data: PIDTuneData) -> None:
        self.setpoint_buffer.append(data.setpoint)
        self.measurement_buffer.append(data.measurement)
        self.error_buffer.append(data.error)

        self.setpoint_curve.setData(self.setpoint_buffer.get_array())
        self.measurement_curve.setData(self.measurement_buffer.get_array())
        self.error_curve.setData(self.error_buffer.get_array())

        self.data_received.emit(data)

    def clear(self) -> None:
        self.setpoint_buffer.clear()
        self.measurement_buffer.clear()
        self.error_buffer.clear()
        self.setpoint_curve.setData([])
        self.measurement_curve.setData([])
        self.error_curve.setData([])


class MultiAxisGraphWidget(QWidget):
    def __init__(self, max_points: int = 500, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.max_points = max_points
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)

        self.plots = {}
        axes = ["Roll", "Pitch", "Yaw"]
        colors = ["#ff4444", "#44ff44", "#4444ff"]

        for i, (axis, color) in enumerate(zip(axes, colors)):
            group = QGroupBox(f"{axis} Rate")
            group_layout = QVBoxLayout(group)

            plot_widget = pg.PlotWidget()
            plot_widget.setBackground("#1a1a2e")
            plot_widget.showGrid(x=True, y=True, alpha=0.3)
            plot_widget.setMaximumHeight(120)
            plot_widget.setYRange(-180, 180)

            setpoint = plot_widget.plot(pen=pg.mkPen(color=color, width=2), name="Set")
            measurement = plot_widget.plot(
                pen=pg.mkPen(color="#ffffff", width=1), name="Meas"
            )

            self.plots[axis] = {
                "widget": plot_widget,
                "setpoint": setpoint,
                "measurement": measurement,
                "setpoint_buf": RollingBuffer(self.max_points),
                "measurement_buf": RollingBuffer(self.max_points),
            }

            group_layout.addWidget(plot_widget)
            layout.addWidget(group)

    def add_data(self, axis: str, setpoint: float, measurement: float) -> None:
        if axis in self.plots:
            plots = self.plots[axis]
            plots["setpoint_buf"].append(setpoint)
            plots["measurement_buf"].append(measurement)
            plots["setpoint"].setData(plots["setpoint_buf"].get_array())
            plots["measurement"].setData(plots["measurement_buf"].get_array())

    def clear(self) -> None:
        for plots in self.plots.values():
            plots["setpoint_buf"].clear()
            plots["measurement_buf"].clear()
            plots["setpoint"].setData([])
            plots["measurement"].setData([])


class PIDTuningGraphs(QWidget):
    tune_data_ready = pyqtSignal(PIDTuneData)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._init_ui()
        self._setup_update_timer()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)

        control_layout = QHBoxLayout()

        view_group = QGroupBox("View Options")
        view_layout = QHBoxLayout(view_group)

        view_layout.addWidget(QLabel("Display:"))
        self.view_combo = QComboBox()
        self.view_combo.addItems(
            [
                "All Axes",
                "Roll Rate",
                "Pitch Rate",
                "Yaw Rate",
                "Roll Att",
                "Pitch Att",
                "Yaw Att",
            ]
        )
        view_layout.addWidget(self.view_combo)

        self.show_setpoint = QCheckBox("Setpoint")
        self.show_setpoint.setChecked(True)
        self.show_setpoint.stateChanged.connect(self._update_curves)
        view_layout.addWidget(self.show_setpoint)

        self.show_measurement = QCheckBox("Measurement")
        self.show_measurement.setChecked(True)
        self.show_measurement.stateChanged.connect(self._update_curves)
        view_layout.addWidget(self.show_measurement)

        self.show_error = QCheckBox("Error")
        self.show_error.setChecked(False)
        self.show_error.stateChanged.connect(self._update_curves)
        view_layout.addWidget(self.show_error)

        control_layout.addWidget(view_group)

        self.clear_all_btn = QPushButton("Clear All")
        self.clear_all_btn.clicked.connect(self.clear_all)
        control_layout.addWidget(self.clear_all_btn)

        self.record_btn = QPushButton("Start Recording")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record_toggle)
        control_layout.addWidget(self.record_btn)

        layout.addLayout(control_layout)

        self.graph_widget = MultiAxisGraphWidget()
        layout.addWidget(self.graph_widget)

        self.single_graph = PIDGraphWidget("Detailed View")
        self.single_graph.setVisible(False)
        layout.addWidget(self.single_graph)

        stats_layout = QGridLayout()

        stats_layout.addWidget(QLabel("Peak Error:"), 0, 0)
        self.peak_error_label = QLabel("0.0")
        stats_layout.addWidget(self.peak_error_label, 0, 1)

        stats_layout.addWidget(QLabel("Avg Error:"), 0, 2)
        self.avg_error_label = QLabel("0.0")
        stats_layout.addWidget(self.avg_error_label, 0, 3)

        stats_layout.addWidget(QLabel("Overshoot:"), 0, 4)
        self.overshoot_label = QLabel("0.0%")
        stats_layout.addWidget(self.overshoot_label, 0, 5)

        stats_layout.addWidget(QLabel("Samples:"), 1, 0)
        self.samples_label = QLabel("0")
        stats_layout.addWidget(self.samples_label, 1, 1)

        stats_layout.addWidget(QLabel("Kp:"), 1, 2)
        self.kp_label = QLabel("0.0")
        stats_layout.addWidget(self.kp_label, 1, 3)

        stats_layout.addWidget(QLabel("Ki:"), 1, 4)
        self.ki_label = QLabel("0.0")
        stats_layout.addWidget(self.ki_label, 1, 5)

        stats_layout.addWidget(QLabel("Kd:"), 2, 0)
        self.kd_label = QLabel("0.0")
        stats_layout.addWidget(self.kd_label, 2, 1)

        layout.addLayout(stats_layout)

    def _setup_update_timer(self) -> None:
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._update_stats)
        self._update_timer.start(100)

        self._error_buffer = []
        self._is_recording = False

    def _update_curves(self) -> None:
        for plots in self.graph_widget.plots.values():
            plots["setpoint"].setVisible(self.show_setpoint.isChecked())
            plots["measurement"].setVisible(self.show_measurement.isChecked())

    def _update_stats(self) -> None:
        if self._error_buffer:
            errors = np.array(self._error_buffer[-500:])
            self.peak_error_label.setText(f"{np.max(np.abs(errors)):.2f}")
            self.avg_error_label.setText(f"{np.mean(np.abs(errors)):.2f}")
        self.samples_label.setText(str(len(self._error_buffer)))

    def add_tune_data(
        self,
        setpoint: float,
        measurement: float,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        axis: int = 0,
    ) -> None:
        error = setpoint - measurement

        data = PIDTuneData(
            timestamp=0.0,
            setpoint=setpoint,
            measurement=measurement,
            error=error,
            kp=kp,
            ki=ki,
            kd=kd,
            output=0.0,
            axis=axis,
        )

        axis_names = ["Roll", "Pitch", "Yaw"]
        if axis < 3:
            self.graph_widget.add_data(axis_names[axis], setpoint, measurement)

        self.single_graph.add_data(data)

        if self._is_recording:
            self._error_buffer.append(error)

        self.kp_label.setText(f"{kp:.4f}")
        self.ki_label.setText(f"{ki:.6f}")
        self.kd_label.setText(f"{kd:.4f}")

        if error != 0:
            overshoot = ((measurement - setpoint) / abs(error)) * 100
            self.overshoot_label.setText(f"{max(0, overshoot):.1f}%")

        self.tune_data_ready.emit(data)

    def clear_all(self) -> None:
        self.graph_widget.clear()
        self.single_graph.clear()
        self._error_buffer.clear()

    def _on_record_toggle(self, checked: bool) -> None:
        self._is_recording = checked
        self.record_btn.setText("Stop Recording" if checked else "Start Recording")

    def start_recording(self) -> None:
        self._is_recording = True
        self._error_buffer.clear()
        self.record_btn.setText("Stop Recording")
        self.record_btn.setChecked(True)

    def stop_recording(self) -> None:
        self._is_recording = False
        self.record_btn.setText("Start Recording")
        self.record_btn.setChecked(False)

    def get_recorded_data(self) -> np.ndarray:
        return np.array(self._error_buffer)
