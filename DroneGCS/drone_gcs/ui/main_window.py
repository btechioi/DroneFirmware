"""Main Window for Drone GCS Application"""

from __future__ import annotations

import logging
import os
import sys
from typing import Optional

from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QPushButton,
    QLabel,
    QComboBox,
    QLineEdit,
    QGroupBox,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QDoubleSpinBox,
    QSpinBox,
    QTextEdit,
    QStatusBar,
    QMenuBar,
    QMenu,
    QMessageBox,
    QSplitter,
    QProgressBar,
    QDial,
    QSlider,
    QCheckBox,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QAction, QPainter, QColor, QBrush, QPen, QKeyEvent

from ..connection import ConnectionManager
from ..protocol import PIDProfile, TuneMethod, TuneAxis, TuneState
from ..control import joystick
from .graphs import PIDTuningGraphs

logger = logging.getLogger(__name__)


class TelemetryThread(QThread):
    message_received = pyqtSignal(object)
    status_changed = pyqtSignal(bool)

    def __init__(self, connection: ConnectionManager) -> None:
        super().__init__()
        self.connection = connection
        self._running = True

    def run(self) -> None:
        while self._running:
            msg = self.connection.protocol.receive_message(timeout=0.1)
            if msg:
                self.message_received.emit(msg)
            self.status_changed.emit(self.connection.is_connected)

    def stop(self) -> None:
        self._running = False


class JoystickMonitorThread(QThread):
    joystick_updated = pyqtSignal(object)

    def __init__(self, controller: joystick.JoystickController) -> None:
        super().__init__()
        self.controller = controller
        self._running = True

    def run(self) -> None:
        import pygame

        while self._running:
            for event in pygame.event.get():
                if (
                    event.type == pygame.JOYAXISMOTION
                    or event.type == pygame.JOYBUTTONUP
                ):
                    state = self.controller.update()
                    if state:
                        self.joystick_updated.emit(state)
            QThread.msleep(10)

    def stop(self) -> None:
        self._running = False


class VirtualJoystickWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.roll = 0.0
        self.pitch = 0.0
        self.throttle = 0.0
        self.yaw = 0.0

    def set_values(self, roll: float, pitch: float, throttle: float, yaw: float):
        self.roll = roll
        self.pitch = pitch
        self.throttle = throttle
        self.yaw = yaw
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2

        painter.fillRect(event.rect(), QColor("#1a1a2e"))

        outer_brush = QBrush(QColor("#2a2a4e"))
        painter.setBrush(outer_brush)
        painter.drawEllipse(cx - 80, cy - 80, 160, 160)

        roll_x = int(self.roll * 60)
        pitch_y = int(self.pitch * 60)

        inner_brush = QBrush(QColor("#00ff88"))
        painter.setBrush(inner_brush)
        painter.drawEllipse(cx + roll_x - 15, cy - pitch_y - 15, 30, 30)

        pen = QPen(QColor("#ffffff"))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawLine(cx - 80, cy, cx + 80, cy)
        painter.drawLine(cx, cy - 80, cx, cy + 80)


class KeyboardController:
    ROLL_STEP = 50
    PITCH_STEP = 50
    YAW_STEP = 50
    THROTTLE_STEP = 25
    THROTTLE_MIN = 1000
    THROTTLE_MAX = 2000
    NEUTRAL = 1500

    def __init__(self) -> None:
        self.roll = self.NEUTRAL
        self.pitch = self.NEUTRAL
        self.throttle = self.THROTTLE_MIN
        self.yaw = self.NEUTRAL
        self.aux1 = 1000
        self.armed = False
        self._keys_pressed: set = set()

    def reset(self) -> None:
        self.roll = self.NEUTRAL
        self.pitch = self.NEUTRAL
        self.throttle = self.THROTTLE_MIN
        self.yaw = self.NEUTRAL
        self._keys_pressed.clear()

    def press_key(self, key: str) -> None:
        self._keys_pressed.add(key)
        self._update()

    def release_key(self, key: str) -> None:
        self._keys_pressed.discard(key)
        self._update()

    def _update(self) -> None:
        if "a" in self._keys_pressed:
            self.roll = max(1000, self.roll - self.ROLL_STEP)
        elif "d" in self._keys_pressed:
            self.roll = min(2000, self.roll + self.ROLL_STEP)
        else:
            self.roll = self.NEUTRAL

        if "w" in self._keys_pressed:
            self.pitch = max(1000, self.pitch - self.PITCH_STEP)
        elif "s" in self._keys_pressed:
            self.pitch = min(2000, self.pitch + self.PITCH_STEP)
        else:
            self.pitch = self.NEUTRAL

        if "q" in self._keys_pressed:
            self.yaw = max(1000, self.yaw - self.YAW_STEP)
        elif "e" in self._keys_pressed:
            self.yaw = min(2000, self.yaw + self.YAW_STEP)
        else:
            self.yaw = self.NEUTRAL

        if " " in self._keys_pressed or "shift" in self._keys_pressed:
            self.throttle = min(self.THROTTLE_MAX, self.throttle + self.THROTTLE_STEP)
        elif "control" in self._keys_pressed:
            self.throttle = max(self.THROTTLE_MIN, self.throttle - self.THROTTLE_STEP)

        self.armed = "return" in self._keys_pressed and self.throttle < 1050


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.connection = ConnectionManager()
        self.joystick_controller = joystick.JoystickController()
        self.keyboard_controller = KeyboardController()
        self.joystick_thread: Optional[JoystickMonitorThread] = None
        self.telemetry_thread: Optional[TelemetryThread] = None
        self.current_profile = PIDProfile.default()
        self.tune_state = TuneState.IDLE
        self._last_joystick_state = None
        self._joystick_enabled = False
        self._keyboard_enabled = True
        self._last_keyboard_state = None

        self._init_ui()
        self._setup_timers()
        self._connect_signals()

        self.setWindowTitle("Drone GCS - Ground Control Station")
        self.setMinimumSize(1400, 900)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def _init_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        menu_bar = self.menuBar()
        file_menu = menu_bar.addMenu("File")
        exit_action = QAction("Exit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        help_menu = menu_bar.addMenu("Help")
        help_menu.addAction("About", self._show_about)

        main_layout.addWidget(self._create_connection_group())
        main_layout.addWidget(self._create_tabs())

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel("Disconnected")
        self.status_bar.addPermanentWidget(self.status_label)
        self.msg_rate_label = QLabel("0 msg/s")
        self.status_bar.addPermanentWidget(self.msg_rate_label)
        self.joystick_label = QLabel("No Joystick")
        self.status_bar.addPermanentWidget(self.joystick_label)
        self.keyboard_status_label = QLabel("KB: Active")
        self.keyboard_status_label.setStyleSheet("color: #00ff88;")
        self.status_bar.addPermanentWidget(self.keyboard_status_label)

    def _create_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("Type:"), 0, 0)
        self.conn_type_combo = QComboBox()
        self.conn_type_combo.addItems(["USB Serial", "UDP", "Relay (USB->Drone)"])
        self.conn_type_combo.currentIndexChanged.connect(self._on_conn_type_changed)
        layout.addWidget(self.conn_type_combo, 0, 1)

        layout.addWidget(QLabel("Port/Host:"), 0, 2)
        self.port_input = QComboBox()
        self.port_input.setEditable(True)
        self.port_input.setPlaceholderText("/dev/ttyUSB0 or 192.168.1.100")
        layout.addWidget(self.port_input, 0, 3)

        layout.addWidget(QLabel("Baudrate:"), 0, 4)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(
            ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]
        )
        self.baud_combo.setCurrentText("115200")
        layout.addWidget(self.baud_combo, 0, 5)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect)
        layout.addWidget(self.connect_btn, 0, 6)

        self.auto_detect_btn = QPushButton("Auto Detect")
        self.auto_detect_btn.clicked.connect(self._on_auto_detect)
        layout.addWidget(self.auto_detect_btn, 0, 7)

        self._refresh_ports()

        return group

    def _create_tabs(self) -> QTabWidget:
        tabs = QTabWidget()
        tabs.addTab(self._create_flight_tab(), "Flight Control")
        tabs.addTab(self._create_control_tab(), "PC Control")
        tabs.addTab(self._create_pid_tab(), "PID Tuning")
        tabs.addTab(self._create_motors_tab(), "Motor Test")
        tabs.addTab(self._create_sensors_tab(), "Sensors")
        tabs.addTab(self._create_messages_tab(), "Messages")
        return tabs

    def _create_flight_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        left_panel = QVBoxLayout()

        mode_group = QGroupBox("Flight Mode")
        mode_layout = QVBoxLayout(mode_group)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(
            [
                "Stabilize",
                "Acro",
                "AltHold",
                "Loiter",
                "RTL",
                "Auto",
                "Guided",
                "Circle",
            ]
        )
        mode_layout.addWidget(self.mode_combo)
        left_panel.addWidget(mode_group)

        arm_group = QGroupBox("Arming")
        arm_layout = QVBoxLayout(arm_group)
        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setStyleSheet(
            "background-color: #ff4444; color: white; font-weight: bold;"
        )
        self.arm_btn.clicked.connect(self._on_arm)
        arm_layout.addWidget(self.arm_btn)

        self.disarm_btn = QPushButton("DISARM")
        self.disarm_btn.setStyleSheet("background-color: #444444; color: white;")
        self.disarm_btn.clicked.connect(self._on_disarm)
        arm_layout.addWidget(self.disarm_btn)
        left_panel.addWidget(arm_group)

        throttle_group = QGroupBox("Throttle")
        throttle_layout = QVBoxLayout(throttle_group)
        self.throttle_slider = QSpinBox()
        self.throttle_slider.setRange(1000, 2000)
        self.throttle_slider.setValue(1000)
        self.throttle_slider.valueChanged.connect(self._on_throttle_change)
        throttle_layout.addWidget(self.throttle_slider)
        left_panel.addWidget(throttle_group)

        layout.addLayout(left_panel)

        center_panel = QVBoxLayout()
        self.attitude_label = QLabel("Attitude: Roll: 0°  Pitch: 0°  Yaw: 0°")
        self.attitude_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        center_panel.addWidget(self.attitude_label)

        self.rates_label = QLabel("Rates: Roll: 0°/s  Pitch: 0°/s  Yaw: 0°/s")
        center_panel.addWidget(self.rates_label)

        self.altitude_label = QLabel("Altitude: 0.0 m")
        self.altitude_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        center_panel.addWidget(self.altitude_label)

        layout.addLayout(center_panel)

        right_panel = QVBoxLayout()
        gps_group = QGroupBox("GPS")
        gps_layout = QGridLayout(gps_group)
        gps_layout.addWidget(QLabel("Lat:"), 0, 0)
        self.lat_label = QLabel("0.0")
        gps_layout.addWidget(self.lat_label, 0, 1)
        gps_layout.addWidget(QLabel("Lon:"), 1, 0)
        self.lon_label = QLabel("0.0")
        gps_layout.addWidget(self.lon_label, 1, 1)
        gps_layout.addWidget(QLabel("Speed:"), 2, 0)
        self.speed_label = QLabel("0.0 m/s")
        gps_layout.addWidget(self.speed_label, 2, 1)
        gps_layout.addWidget(QLabel("Sats:"), 3, 0)
        self.sats_label = QLabel("0")
        gps_layout.addWidget(self.sats_label, 3, 1)
        right_panel.addWidget(gps_group)

        batt_group = QGroupBox("Battery")
        batt_layout = QGridLayout(batt_group)
        batt_layout.addWidget(QLabel("Voltage:"), 0, 0)
        self.voltage_label = QLabel("0.0 V")
        batt_layout.addWidget(self.voltage_label, 0, 1)
        batt_layout.addWidget(QLabel("Current:"), 1, 0)
        self.current_label = QLabel("0.0 A")
        batt_layout.addWidget(self.current_label, 1, 1)
        right_panel.addWidget(batt_group)

        layout.addLayout(right_panel)

        return tab

    def _create_control_tab(self) -> QWidget:
        tab = QWidget()
        layout = QHBoxLayout(tab)

        left_panel = QVBoxLayout()

        js_select_group = QGroupBox("Joystick Selection")
        js_select_layout = QVBoxLayout(js_select_group)

        self.js_refresh_btn = QPushButton("Refresh Joysticks")
        self.js_refresh_btn.clicked.connect(self._refresh_joysticks)
        js_select_layout.addWidget(self.js_refresh_btn)

        self.js_combo = QComboBox()
        self.js_combo.currentIndexChanged.connect(self._on_js_selected)
        js_select_layout.addWidget(self.js_combo)

        self.js_enable_btn = QPushButton("Enable Joystick Control")
        self.js_enable_btn.setCheckable(True)
        self.js_enable_btn.clicked.connect(self._on_js_enable_toggle)
        self.js_enable_btn.setEnabled(False)
        js_select_layout.addWidget(self.js_enable_btn)

        left_panel.addWidget(js_select_group)

        settings_group = QGroupBox("Settings")
        settings_layout = QGridLayout(settings_group)

        settings_layout.addWidget(QLabel("Deadzone:"), 0, 0)
        self.deadzone_spin = QDoubleSpinBox()
        self.deadzone_spin.setRange(0.01, 0.5)
        self.deadzone_spin.setSingleStep(0.01)
        self.deadzone_spin.setValue(0.1)
        self.deadzone_spin.valueChanged.connect(self._on_deadzone_change)
        settings_layout.addWidget(self.deadzone_spin, 0, 1)

        settings_layout.addWidget(QLabel("Expo:"), 1, 0)
        self.expo_spin = QDoubleSpinBox()
        self.expo_spin.setRange(0.0, 1.0)
        self.expo_spin.setSingleStep(0.05)
        self.expo_spin.setValue(0.2)
        self.expo_spin.valueChanged.connect(self._on_expo_change)
        settings_layout.addWidget(self.expo_spin, 1, 1)

        self.throttle_curve_cb = QCheckBox("Throttle Curve")
        self.throttle_curve_cb.setChecked(True)
        self.throttle_curve_cb.stateChanged.connect(self._on_throttle_curve_change)
        settings_layout.addWidget(self.throttle_curve_cb, 2, 0, 1, 2)

        left_panel.addWidget(settings_group)

        info_group = QGroupBox("Status")
        info_layout = QGridLayout(info_group)

        info_layout.addWidget(QLabel("Roll:"), 0, 0)
        self.ctrl_roll_label = QLabel("1500")
        info_layout.addWidget(self.ctrl_roll_label, 0, 1)

        info_layout.addWidget(QLabel("Pitch:"), 1, 0)
        self.ctrl_pitch_label = QLabel("1500")
        info_layout.addWidget(self.ctrl_pitch_label, 1, 1)

        info_layout.addWidget(QLabel("Throttle:"), 2, 0)
        self.ctrl_throttle_label = QLabel("1000")
        info_layout.addWidget(self.ctrl_throttle_label, 2, 1)

        info_layout.addWidget(QLabel("Yaw:"), 3, 0)
        self.ctrl_yaw_label = QLabel("1500")
        info_layout.addWidget(self.ctrl_yaw_label, 3, 1)

        info_layout.addWidget(QLabel("Armed:"), 4, 0)
        self.ctrl_armed_label = QLabel("NO")
        info_layout.addWidget(self.ctrl_armed_label, 4, 1)

        left_panel.addWidget(info_group)

        kb_group = QGroupBox("Keyboard Control")
        kb_layout = QVBoxLayout(kb_group)
        kb_layout.addWidget(QLabel("W/S - Pitch Forward/Back"))
        kb_layout.addWidget(QLabel("A/D - Roll Left/Right"))
        kb_layout.addWidget(QLabel("Q/E - Yaw Left/Right"))
        kb_layout.addWidget(QLabel("Space - Throttle Up"))
        kb_layout.addWidget(QLabel("Ctrl - Throttle Down"))
        kb_layout.addWidget(QLabel("Enter - ARM (throttle low)"))
        kb_layout.addWidget(QLabel("Escape - DISARM"))
        kb_layout.addWidget(QLabel("R - Reset Controls"))

        self.kb_enable_cb = QCheckBox("Enable Keyboard Control")
        self.kb_enable_cb.setChecked(True)
        kb_layout.addWidget(self.kb_enable_cb)

        self.kb_status_label = QLabel("Keyboard: Active")
        self.kb_status_label.setStyleSheet("color: #00ff88; font-weight: bold;")
        kb_layout.addWidget(self.kb_status_label)

        left_panel.addWidget(kb_group)

        layout.addLayout(left_panel)

        center_panel = QVBoxLayout()
        center_panel.addWidget(
            QLabel(
                "Virtual Joystick (Roll/Pitch)", alignment=Qt.AlignmentFlag.AlignCenter
            )
        )
        self.virtual_joystick = VirtualJoystickWidget()
        center_panel.addWidget(self.virtual_joystick)

        throttle_group = QGroupBox("Throttle")
        throttle_layout = QVBoxLayout(throttle_group)
        self.ctrl_throttle_slider = QSlider(Qt.Orientation.Vertical)
        self.ctrl_throttle_slider.setRange(1000, 2000)
        self.ctrl_throttle_slider.setValue(1000)
        self.ctrl_throttle_slider.valueChanged.connect(self._on_ctrl_throttle_change)
        throttle_layout.addWidget(self.ctrl_throttle_slider)
        self.ctrl_throttle_value = QLabel("1000")
        throttle_layout.addWidget(self.ctrl_throttle_value)
        center_panel.addWidget(throttle_group)

        layout.addLayout(center_panel)

        self._refresh_joysticks()

        return tab

    def _create_pid_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        control_layout = QHBoxLayout()

        tune_group = QGroupBox("Auto-Tune")
        tune_layout = QGridLayout(tune_group)

        tune_layout.addWidget(QLabel("Method:"), 0, 0)
        self.tune_method_combo = QComboBox()
        self.tune_method_combo.addItems(
            ["Ziegler-Nichols", "Relay", "Step Response", "Frequency Sweep"]
        )
        tune_layout.addWidget(self.tune_method_combo, 0, 1)

        tune_layout.addWidget(QLabel("Axis:"), 1, 0)
        self.tune_axis_combo = QComboBox()
        self.tune_axis_combo.addItems(
            [
                "Roll Rate",
                "Pitch Rate",
                "Yaw Rate",
                "Roll Attitude",
                "Pitch Attitude",
                "Yaw Attitude",
                "Altitude",
                "Position",
            ]
        )
        tune_layout.addWidget(self.tune_axis_combo, 1, 1)

        tune_layout.addWidget(QLabel("Amplitude:"), 2, 0)
        self.tune_amplitude = QDoubleSpinBox()
        self.tune_amplitude.setRange(10, 200)
        self.tune_amplitude.setValue(50)
        tune_layout.addWidget(self.tune_amplitude, 2, 1)

        self.start_tune_btn = QPushButton("Start Tuning")
        self.start_tune_btn.clicked.connect(self._on_start_tune)
        tune_layout.addWidget(self.start_tune_btn, 3, 0)

        self.stop_tune_btn = QPushButton("Stop Tuning")
        self.stop_tune_btn.clicked.connect(self._on_stop_tune)
        self.stop_tune_btn.setEnabled(False)
        tune_layout.addWidget(self.stop_tune_btn, 3, 1)

        self.tune_progress = QProgressBar()
        tune_layout.addWidget(self.tune_progress, 4, 0, 1, 2)

        self.tune_status_label = QLabel("Status: Idle")
        tune_layout.addWidget(self.tune_status_label, 5, 0, 1, 2)

        control_layout.addWidget(tune_group)

        profile_group = QGroupBox("PID Profiles")
        profile_layout = QVBoxLayout(profile_group)

        profile_btn_layout = QHBoxLayout()
        for i in range(4):
            btn = QPushButton(f"Profile {i}")
            btn.clicked.connect(lambda checked, slot=i: self._on_load_profile(slot))
            profile_btn_layout.addWidget(btn)
        profile_layout.addLayout(profile_btn_layout)

        save_btn_layout = QHBoxLayout()
        for i in range(4):
            btn = QPushButton(f"Save {i}")
            btn.clicked.connect(lambda checked, slot=i: self._on_save_profile(slot))
            save_btn_layout.addWidget(btn)
        profile_layout.addLayout(save_btn_layout)

        self.reset_defaults_btn = QPushButton("Reset to Defaults")
        self.reset_defaults_btn.clicked.connect(self._on_reset_defaults)
        profile_layout.addWidget(self.reset_defaults_btn)

        control_layout.addWidget(profile_group)

        layout.addLayout(control_layout)

        self.pid_graphs = PIDTuningGraphs()
        layout.addWidget(self.pid_graphs)

        gains_group = QGroupBox("PID Gains")
        gains_layout = QGridLayout(gains_group)

        gains_layout.addWidget(QLabel("Axis"), 0, 0)
        self.pid_axis_combo = QComboBox()
        self.pid_axis_combo.addItems(
            ["Roll Rate", "Pitch Rate", "Yaw Rate", "Roll Att", "Pitch Att", "Yaw Att"]
        )
        self.pid_axis_combo.currentIndexChanged.connect(self._on_pid_axis_change)
        gains_layout.addWidget(self.pid_axis_combo, 0, 1)

        gains_layout.addWidget(QLabel("Kp:"), 1, 0)
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0, 100)
        self.kp_spin.setDecimals(4)
        self.kp_spin.setSingleStep(0.01)
        self.kp_spin.valueChanged.connect(self._on_pid_gain_change)
        gains_layout.addWidget(self.kp_spin, 1, 1)

        gains_layout.addWidget(QLabel("Ki:"), 2, 0)
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0, 100)
        self.ki_spin.setDecimals(6)
        self.ki_spin.setSingleStep(0.001)
        self.ki_spin.valueChanged.connect(self._on_pid_gain_change)
        gains_layout.addWidget(self.ki_spin, 2, 1)

        gains_layout.addWidget(QLabel("Kd:"), 3, 0)
        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0, 100)
        self.kd_spin.setDecimals(4)
        self.kd_spin.setSingleStep(0.01)
        self.kd_spin.valueChanged.connect(self._on_pid_gain_change)
        gains_layout.addWidget(self.kd_spin, 3, 1)

        self.apply_gains_btn = QPushButton("Apply Gains")
        self.apply_gains_btn.clicked.connect(self._on_apply_gains)
        gains_layout.addWidget(self.apply_gains_btn, 4, 0, 1, 2)

        layout.addWidget(gains_group)

        return tab

    def _create_motors_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        info_label = QLabel(
            "Motor Test - Disarm before testing. Props OFF recommended!"
        )
        info_label.setStyleSheet("color: orange; font-weight: bold;")
        layout.addWidget(info_label)

        grid = QGridLayout()
        self.motor_spins = []
        for i in range(4):
            grid.addWidget(QLabel(f"Motor {i + 1}:"), i, 0)
            spin = QSpinBox()
            spin.setRange(1000, 2000)
            spin.setValue(1000)
            spin.valueChanged.connect(self._on_motor_change)
            grid.addWidget(spin, i, 1)
            self.motor_spins.append(spin)

        layout.addLayout(grid)

        btn_layout = QHBoxLayout()
        self.test_all_btn = QPushButton("Test All Motors")
        self.test_all_btn.clicked.connect(self._on_test_all_motors)
        btn_layout.addWidget(self.test_all_btn)

        self.stop_motors_btn = QPushButton("STOP ALL MOTORS")
        self.stop_motors_btn.setStyleSheet(
            "background-color: red; color: white; font-weight: bold;"
        )
        self.stop_motors_btn.clicked.connect(self._on_stop_all_motors)
        btn_layout.addWidget(self.stop_motors_btn)

        layout.addLayout(btn_layout)

        return tab

    def _create_sensors_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        imu_group = QGroupBox("IMU")
        imu_layout = QGridLayout(imu_group)
        imu_layout.addWidget(QLabel("Accel X:"), 0, 0)
        self.accel_x = QLabel("0.0")
        imu_layout.addWidget(self.accel_x, 0, 1)
        imu_layout.addWidget(QLabel("Accel Y:"), 1, 0)
        self.accel_y = QLabel("0.0")
        imu_layout.addWidget(self.accel_y, 1, 1)
        imu_layout.addWidget(QLabel("Accel Z:"), 2, 0)
        self.accel_z = QLabel("0.0")
        imu_layout.addWidget(self.accel_z, 2, 1)
        imu_layout.addWidget(QLabel("Gyro X:"), 3, 0)
        self.gyro_x = QLabel("0.0")
        imu_layout.addWidget(self.gyro_x, 3, 1)
        imu_layout.addWidget(QLabel("Gyro Y:"), 4, 0)
        self.gyro_y = QLabel("0.0")
        imu_layout.addWidget(self.gyro_y, 4, 1)
        imu_layout.addWidget(QLabel("Gyro Z:"), 5, 0)
        self.gyro_z = QLabel("0.0")
        imu_layout.addWidget(self.gyro_z, 5, 1)
        layout.addWidget(imu_group)

        baro_group = QGroupBox("Barometer")
        baro_layout = QGridLayout(baro_group)
        baro_layout.addWidget(QLabel("Temperature:"), 0, 0)
        self.temp_label = QLabel("0.0 C")
        baro_layout.addWidget(self.temp_label, 0, 1)
        baro_layout.addWidget(QLabel("Pressure:"), 1, 0)
        self.pressure_label = QLabel("0.0 hPa")
        baro_layout.addWidget(self.pressure_label, 1, 1)
        layout.addWidget(baro_group)

        return tab

    def _create_messages_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.message_log = QTextEdit()
        self.message_log.setReadOnly(True)
        layout.addWidget(self.message_log)

        return tab

    def _setup_timers(self) -> None:
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(1000)

        self.joystick_send_timer = QTimer()
        self.joystick_send_timer.timeout.connect(self._send_control_input)

        self.keyboard_send_timer = QTimer()
        self.keyboard_send_timer.timeout.connect(self._send_keyboard_control)
        self.keyboard_send_timer.start(50)

    def _connect_signals(self) -> None:
        self.connection.set_message_callback(self._on_message)
        self.connection.set_status_callback(self._on_connection_status)

        self.joystick_controller.set_state_callback(self._on_joystick_state)

    def _refresh_ports(self) -> None:
        ports = self.connection.list_serial_ports()
        self.port_input.clear()
        if ports:
            self.port_input.addItems(ports)
        else:
            self.port_input.addItem("/dev/ttyUSB0")
            self.port_input.addItem("COM3")

    def _refresh_joysticks(self) -> None:
        self.js_combo.clear()

        if not joystick.has_pygame():
            self.js_combo.addItem("pygame-ce not installed")
            self.js_enable_btn.setEnabled(False)
            return

        joysticks = self.joystick_controller.get_joysticks()
        if not joysticks:
            self.js_combo.addItem("No joysticks found")
            self.js_enable_btn.setEnabled(False)
        else:
            for js in joysticks:
                self.js_combo.addItem(
                    f"{js['name']} ({js['num_axes']} axes, {js['num_buttons']} buttons)"
                )
            self.js_enable_btn.setEnabled(True)

    def _on_conn_type_changed(self, index: int) -> None:
        if index == 1:
            self.port_input.clear()
            self.port_input.addItem("0.0.0.0")
            self.baud_combo.setCurrentText("14550")
            self.baud_combo.setEditable(True)
        else:
            self.baud_combo.setEditable(False)
            self.baud_combo.setCurrentText("115200")
            self._refresh_ports()

    def _on_auto_detect(self) -> None:
        device = self.connection.detect_drone_device()
        if device:
            self.port_input.setCurrentText(device.port)
            self.status_bar.showMessage(
                f"Auto-detected: {device.description} on {device.port}"
            )
        else:
            self.status_bar.showMessage("No drone device detected")

    def _on_connect(self) -> None:
        if self.connection.is_connected:
            self.connection.disconnect()
            self.connect_btn.setText("Connect")
            self.status_label.setText("Disconnected")
            if self.telemetry_thread:
                self.telemetry_thread.stop()
                self.telemetry_thread = None
        else:
            port = self.port_input.currentText()
            baud = int(self.baud_combo.currentText())
            conn_type = self.conn_type_combo.currentIndex()

            success = False
            if conn_type == 0:
                success = self.connection.connect_usb(port, baud)
            elif conn_type == 1:
                success = self.connection.connect_udp("0.0.0.0", 14550)
            elif conn_type == 2:
                success = self.connection.connect_relay(port, port, baud)

            if success:
                self.connect_btn.setText("Disconnect")
                self.status_label.setText(f"Connected: {port}")
                self.telemetry_thread = TelemetryThread(self.connection)
                self.telemetry_thread.message_received.connect(self._on_message)
                self.telemetry_thread.status_changed.connect(self._on_connection_status)
                self.telemetry_thread.start()
            else:
                QMessageBox.warning(
                    self, "Connection Failed", f"Could not connect to {port}"
                )

    def _on_js_selected(self, index: int) -> None:
        if index >= 0 and self.joystick_controller.num_joysticks > 0:
            self.joystick_controller.select_joystick(index)

    def _on_js_enable_toggle(self, checked: bool) -> None:
        if checked:
            if self.joystick_controller.select_joystick(self.js_combo.currentIndex()):
                self.joystick_thread = JoystickMonitorThread(self.joystick_controller)
                self.joystick_thread.joystick_updated.connect(self._on_joystick_input)
                self.joystick_thread.start()
                self._joystick_enabled = True
                self.js_enable_btn.setText("Disable Joystick Control")
                self.joystick_send_timer.start(50)
                self.status_bar.showMessage(
                    "Joystick control enabled - ARM with button 7 (throttle low)"
                )
        else:
            if self.joystick_thread:
                self.joystick_thread.stop()
                self.joystick_thread = None
            self._joystick_enabled = False
            self.js_enable_btn.setText("Enable Joystick Control")
            self.joystick_send_timer.stop()
            self.status_bar.showMessage("Joystick control disabled")

    def _on_joystick_input(self, state: joystick.JoystickState) -> None:
        self._last_joystick_state = state
        self.ctrl_roll_label.setText(str(state.roll))
        self.ctrl_pitch_label.setText(str(state.pitch))
        self.ctrl_throttle_label.setText(str(state.throttle))
        self.ctrl_yaw_label.setText(str(state.yaw))
        self.ctrl_armed_label.setText("YES" if state.armed else "NO")
        self.ctrl_armed_label.setStyleSheet(
            "color: green; font-weight: bold;" if state.armed else "color: gray;"
        )

        roll_norm = (state.roll - 1500) / 500.0
        pitch_norm = (state.pitch - 1500) / 500.0
        throttle_norm = (state.throttle - 1000) / 1000.0
        yaw_norm = (state.yaw - 1500) / 500.0

        self.virtual_joystick.set_values(roll_norm, pitch_norm, throttle_norm, yaw_norm)
        self.ctrl_throttle_slider.blockSignals(True)
        self.ctrl_throttle_slider.setValue(state.throttle)
        self.ctrl_throttle_slider.blockSignals(False)
        self.ctrl_throttle_value.setText(str(state.throttle))

    def _send_control_input(self) -> None:
        if self._joystick_enabled and self._last_joystick_state:
            state = self._last_joystick_state
            self.connection.send_rc_channels(
                state.roll,
                state.pitch,
                state.throttle,
                state.yaw,
                state.aux1,
                state.aux2,
                state.aux3,
                state.aux4,
            )

    def _send_keyboard_control(self) -> None:
        if not self._keyboard_enabled:
            return

        kb = self.keyboard_controller
        self.connection.send_rc_channels(
            kb.roll,
            kb.pitch,
            kb.throttle,
            kb.yaw,
            2000 if kb.armed else 1000,
            1000,
            1000,
            1000,
        )

        if self._last_keyboard_state != (kb.roll, kb.pitch, kb.throttle, kb.yaw):
            self._last_keyboard_state = (kb.roll, kb.pitch, kb.throttle, kb.yaw)
            self._update_keyboard_display(kb)

    def _update_keyboard_display(self, kb: KeyboardController) -> None:
        self.ctrl_roll_label.setText(str(kb.roll))
        self.ctrl_pitch_label.setText(str(kb.pitch))
        self.ctrl_throttle_label.setText(str(kb.throttle))
        self.ctrl_yaw_label.setText(str(kb.yaw))
        self.ctrl_armed_label.setText("YES" if kb.armed else "NO")
        self.ctrl_armed_label.setStyleSheet(
            "color: green; font-weight: bold;" if kb.armed else "color: gray;"
        )

        roll_norm = (kb.roll - 1500) / 500.0
        pitch_norm = (kb.pitch - 1500) / 500.0
        throttle_norm = (kb.throttle - 1000) / 1000.0
        yaw_norm = (kb.yaw - 1500) / 500.0
        self.virtual_joystick.set_values(roll_norm, pitch_norm, throttle_norm, yaw_norm)

        self.ctrl_throttle_slider.blockSignals(True)
        self.ctrl_throttle_slider.setValue(kb.throttle)
        self.ctrl_throttle_slider.blockSignals(False)
        self.ctrl_throttle_value.setText(str(kb.throttle))

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if not self._keyboard_enabled:
            super().keyPressEvent(event)
            return

        key = event.key()
        key_name = ""

        if key == Qt.Key.Key_W:
            key_name = "w"
        elif key == Qt.Key.Key_S:
            key_name = "s"
        elif key == Qt.Key.Key_A:
            key_name = "a"
        elif key == Qt.Key.Key_D:
            key_name = "d"
        elif key == Qt.Key.Key_Q:
            key_name = "q"
        elif key == Qt.Key.Key_E:
            key_name = "e"
        elif key == Qt.Key.Key_Space:
            key_name = " "
        elif key == Qt.Key.Key_Control:
            key_name = "control"
        elif key == Qt.Key.Key_Shift:
            key_name = "shift"
        elif key == Qt.Key.Key_Return or key == Qt.Key.Key_Enter:
            key_name = "return"
        elif key == Qt.Key.Key_Escape:
            self._on_disarm()
            self.keyboard_controller.reset()
            self._update_keyboard_display(self.keyboard_controller)
            return
        elif key == Qt.Key.Key_R:
            self.keyboard_controller.reset()
            self._update_keyboard_display(self.keyboard_controller)
            return
        else:
            super().keyPressEvent(event)
            return

        self.keyboard_controller.press_key(key_name)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        if not self._keyboard_enabled:
            super().keyReleaseEvent(event)
            return

        key = event.key()
        key_name = ""

        if key == Qt.Key.Key_W:
            key_name = "w"
        elif key == Qt.Key.Key_S:
            key_name = "s"
        elif key == Qt.Key.Key_A:
            key_name = "a"
        elif key == Qt.Key.Key_D:
            key_name = "d"
        elif key == Qt.Key.Key_Q:
            key_name = "q"
        elif key == Qt.Key.Key_E:
            key_name = "e"
        elif key == Qt.Key.Key_Space:
            key_name = " "
        elif key == Qt.Key.Key_Control:
            key_name = "control"
        elif key == Qt.Key.Key_Shift:
            key_name = "shift"
        elif key == Qt.Key.Key_Return or key == Qt.Key.Key_Enter:
            key_name = "return"
        else:
            super().keyReleaseEvent(event)
            return

        self.keyboard_controller.release_key(key_name)
        super().keyReleaseEvent(event)

    def _on_deadzone_change(self, value: float) -> None:
        self.joystick_controller.DEAD_ZONE = value

    def _on_expo_change(self, value: float) -> None:
        self.joystick_controller.set_expo(value)

    def _on_throttle_curve_change(self, state: int) -> None:
        self.joystick_controller.set_throttle_curve(
            state == Qt.CheckState.Checked.value
        )

    def _on_ctrl_throttle_change(self, value: int) -> None:
        if not self._joystick_enabled and self._last_joystick_state:
            self._last_joystick_state.throttle = value
            self.ctrl_throttle_value.setText(str(value))

    def _on_arm(self) -> None:
        self.connection.send_rc_channels(1500, 1500, 1000, 1500, 2000, 1000, 1000, 1000)
        self.status_bar.showMessage("Motors ARMED - Props may spin!")

    def _on_disarm(self) -> None:
        self.connection.send_rc_channels(1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000)
        self.status_bar.showMessage("Motors DISARMED")

    def _on_throttle_change(self, value: int) -> None:
        self.connection.send_rc_channels(1500, 1500, value, 1500)

    def _on_start_tune(self) -> None:
        method = self.tune_method_combo.currentIndex()
        axis = self.tune_axis_combo.currentIndex()
        amplitude = self.tune_amplitude.value()
        self.connection.send_pid_tune_start(method, axis, amplitude)
        self.start_tune_btn.setEnabled(False)
        self.stop_tune_btn.setEnabled(True)
        self.tune_state = TuneState.COLLECTING_DATA
        self.pid_graphs.start_recording()

    def _on_stop_tune(self) -> None:
        self.connection.send_pid_tune_stop()
        self.start_tune_btn.setEnabled(True)
        self.stop_tune_btn.setEnabled(False)
        self.tune_state = TuneState.IDLE
        self.pid_graphs.stop_recording()

    def _on_load_profile(self, slot: int) -> None:
        self.connection.load_pid_profile(slot)
        self._log_message(f"Loaded profile {slot}")

    def _on_save_profile(self, slot: int) -> None:
        self.connection.save_pid_profile(slot)
        self._log_message(f"Saved profile {slot}")

    def _on_reset_defaults(self) -> None:
        self.current_profile = PIDProfile.default()
        self._on_pid_axis_change(0)

    def _on_pid_axis_change(self, index: int) -> None:
        profile = self.current_profile
        gains = [
            (profile.roll_rate_kp, profile.roll_rate_ki, profile.roll_rate_kd),
            (profile.pitch_rate_kp, profile.pitch_rate_ki, profile.pitch_rate_kd),
            (profile.yaw_rate_kp, profile.yaw_rate_ki, profile.yaw_rate_kd),
            (profile.roll_att_kp, profile.roll_att_ki, profile.roll_att_kd),
            (profile.pitch_att_kp, profile.pitch_att_ki, profile.pitch_att_kd),
            (profile.yaw_att_kp, profile.yaw_att_ki, profile.yaw_att_kd),
        ][index]
        self.kp_spin.setValue(gains[0])
        self.ki_spin.setValue(gains[1])
        self.kd_spin.setValue(gains[2])

    def _on_pid_gain_change(self) -> None:
        pass

    def _on_apply_gains(self) -> None:
        axis = self.pid_axis_combo.currentIndex()
        kp = self.kp_spin.value()
        ki = self.ki_spin.value()
        kd = self.kd_spin.value()
        self.connection.send_pid_gains(axis, kp, ki, kd)
        self._log_message(f"Sent gains for axis {axis}: Kp={kp}, Ki={ki}, Kd={kd}")

    def _on_motor_change(self) -> None:
        for i, spin in enumerate(self.motor_spins):
            self.connection.send_rc_channels(
                1500, 1500, spin.value(), 1500, 1000, 1000, 1000, 1000
            )

    def _on_test_all_motors(self) -> None:
        for i, spin in enumerate(self.motor_spins):
            spin.setValue(1200)
        QTimer.singleShot(3000, self._on_stop_all_motors)

    def _on_stop_all_motors(self) -> None:
        for spin in self.motor_spins:
            spin.setValue(1000)

    def _on_message(self, msg: object) -> None:
        msg_name = getattr(msg, "name", "unknown")
        if msg_name == "ATTITUDE":
            roll = getattr(msg, "roll", 0) * 57.2958
            pitch = getattr(msg, "pitch", 0) * 57.2958
            yaw = getattr(msg, "yaw", 0) * 57.2958
            self.attitude_label.setText(
                f"Attitude: Roll: {roll:.1f}deg  Pitch: {pitch:.1f}deg  Yaw: {yaw:.1f}deg"
            )

            self.pid_graphs.add_tune_data(
                setpoint=roll,
                measurement=roll,
                kp=self.kp_spin.value(),
                ki=self.ki_spin.value(),
                kd=self.kd_spin.value(),
                axis=0,
            )
        elif msg_name == "GLOBAL_POSITION_INT":
            alt = getattr(msg, "relative_alt", 0) / 1000.0
            self.altitude_label.setText(f"Altitude: {alt:.1f} m")
        elif msg_name == "RAW_IMU":
            accel_x = getattr(msg, "xacc", 0) / 1000.0
            accel_y = getattr(msg, "yacc", 0) / 1000.0
            accel_z = getattr(msg, "zacc", 0) / 1000.0
            gyro_x = getattr(msg, "xgyro", 0) / 1000.0
            gyro_y = getattr(msg, "ygyro", 0) / 1000.0
            gyro_z = getattr(msg, "zgyro", 0) / 1000.0
            self.accel_x.setText(f"{accel_x:.2f}")
            self.accel_y.setText(f"{accel_y:.2f}")
            self.accel_z.setText(f"{accel_z:.2f}")
            self.gyro_x.setText(f"{gyro_x:.2f}")
            self.gyro_y.setText(f"{gyro_y:.2f}")
            self.gyro_z.setText(f"{gyro_z:.2f}")

    def _on_connection_status(self, connected: bool) -> None:
        if connected:
            self.status_label.setText("Connected")
            self.connect_btn.setText("Disconnect")
        else:
            self.status_label.setText("Disconnected")
            self.connect_btn.setText("Connect")

    def _update_status(self) -> None:
        stats = self.connection.get_stats()
        msg_count = sum(stats.get("msg_count", {}).values())
        self.msg_rate_label.setText(f"{msg_count} msgs")

        if self._joystick_enabled and self.joystick_controller.has_joystick():
            self.joystick_label.setText(
                f"JS: {self.joystick_controller.joystick_name[:20]}"
            )
        else:
            self.joystick_label.setText("No Joystick")

    def _log_message(self, text: str) -> None:
        self.message_log.append(text)

    def _show_about(self) -> None:
        QMessageBox.about(
            self,
            "About Drone GCS",
            "Drone GCS v1.0.0\n\nGround Control Station for Drone Firmware\n"
            "Supports Mavlink protocol, PC joystick control, and PID tuning graphs.",
        )

    def closeEvent(self, event) -> None:
        if self.telemetry_thread:
            self.telemetry_thread.stop()
        if self.joystick_thread:
            self.joystick_thread.stop()
        self.joystick_controller.disconnect()
        self.connection.disconnect()
        event.accept()
