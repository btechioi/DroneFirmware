"""Autopilot control for companion computer - handles autonomous flight."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Tuple

logger = logging.getLogger(__name__)


class FlightMode(Enum):
    MANUAL = 0
    STABILIZE = 1
    ALTHOLD = 2
    POSITION_HOLD = 3
    WAYPOINT = 4
    RTL = 5
    TAKEOFF = 6
    LAND = 7


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    tolerance: float = 0.5
    wait_time: float = 0.0


@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class RCChannels:
    roll: int = 1500
    pitch: int = 1500
    throttle: int = 1000
    yaw: int = 1500
    aux1: int = 1000
    aux2: int = 1000
    aux3: int = 1000
    aux4: int = 1000


class Autopilot:
    def __init__(self) -> None:
        self._mode = FlightMode.MANUAL
        self._position = Position()
        self._target = Position()
        self._home_position = Position()

        self._waypoints: List[Waypoint] = []
        self._current_waypoint = 0
        self._waypoint_start_time = 0.0

        self._max_speed = 2.0
        self._takeoff_altitude = 1.5
        self._landing_speed = 0.3

        self._alt_pid = _PID(1.5, 0.02, 0.5)
        self._pos_x_pid = _PID(2.0, 0.1, 0.5)
        self._pos_y_pid = _PID(2.0, 0.1, 0.5)

    @property
    def mode(self) -> FlightMode:
        return self._mode

    def set_mode(self, mode: FlightMode) -> None:
        if mode != self._mode:
            logger.info(f"Autopilot: {self._mode.name} -> {mode.name}")
            self._mode = mode
            self._on_mode_change()

    def _on_mode_change(self) -> None:
        self._alt_pid.reset()
        self._pos_x_pid.reset()
        self._pos_y_pid.reset()

        if self._mode == FlightMode.TAKEOFF:
            self._target.z = self._takeoff_altitude
        elif self._mode == FlightMode.LAND:
            self._target.z = 0.0
        elif self._mode == FlightMode.WAYPOINT and self._waypoints:
            self._current_waypoint = 0
            self._goto_waypoint(0)

    def set_position(self, x: float, y: float, z: float) -> None:
        self._position.x = x
        self._position.y = y
        self._position.z = z

    def set_target(self, x: float, y: float, z: float) -> None:
        self._target.x = x
        self._target.y = y
        self._target.z = z

    def set_home(self) -> None:
        self._home_position.x = self._position.x
        self._home_position.y = self._position.y
        self._home_position.z = self._position.z
        logger.info(
            f"Home set: ({self._home_position.x}, {self._home_position.y}, {self._home_position.z})"
        )

    def load_mission(self, waypoints: List[Waypoint]) -> None:
        self._waypoints = waypoints
        self._current_waypoint = 0
        logger.info(f"Mission loaded: {len(waypoints)} waypoints")

    def _goto_waypoint(self, index: int) -> None:
        if index >= len(self._waypoints):
            logger.info("Mission complete")
            self.set_mode(FlightMode.POSITION_HOLD)
            return

        wp = self._waypoints[index]
        self._target.x = wp.x
        self._target.y = wp.y
        self._target.z = wp.z
        self._waypoint_start_time = time.time()
        logger.info(f"WPT {index}: ({wp.x}, {wp.y}, {wp.z})")

    def compute_control(self) -> RCChannels:
        channels = RCChannels(
            roll=1500, pitch=1500, throttle=1500, yaw=1500, aux1=2000, aux2=1000
        )

        if self._mode == FlightMode.MANUAL:
            channels.aux1 = 1000
            return channels

        channels.aux1 = 2000

        if self._mode == FlightMode.STABILIZE:
            channels.throttle = 1500

        elif self._mode == FlightMode.ALTHOLD:
            channels.throttle = int(
                1500 + self._alt_pid.compute(self._target.z, self._position.z)
            )

        elif self._mode == FlightMode.POSITION_HOLD:
            channels.pitch = int(
                1500 + self._pos_x_pid.compute(self._target.x, self._position.x)
            )
            channels.roll = int(
                1500 + self._pos_y_pid.compute(self._target.y, self._position.y)
            )
            channels.throttle = int(
                1500 + self._alt_pid.compute(self._target.z, self._position.z)
            )

        elif self._mode == FlightMode.WAYPOINT:
            if self._current_waypoint < len(self._waypoints):
                wp = self._waypoints[self._current_waypoint]
                dx = wp.x - self._position.x
                dy = wp.y - self._position.y
                dist = (dx**2 + dy**2) ** 0.5

                if dist < wp.tolerance:
                    if time.time() - self._waypoint_start_time > wp.wait_time:
                        self._current_waypoint += 1
                        self._goto_waypoint(self._current_waypoint)
                else:
                    dx_n = dx / max(dist, 0.001)
                    dy_n = dy / max(dist, 0.001)
                    speed = min(self._max_speed, dist)
                    channels.pitch = 1500 + int(dx_n * speed * 200)
                    channels.roll = 1500 + int(dy_n * speed * 200)

                channels.throttle = int(
                    1500 + self._alt_pid.compute(wp.z, self._position.z)
                )

        elif self._mode == FlightMode.RTL:
            channels.pitch = int(
                1500 + self._pos_x_pid.compute(self._home_position.x, self._position.x)
            )
            channels.roll = int(
                1500 + self._pos_y_pid.compute(self._home_position.y, self._position.y)
            )
            channels.throttle = int(1500 + self._alt_pid.compute(2.0, self._position.z))

        elif self._mode == FlightMode.TAKEOFF:
            if self._position.z < self._takeoff_altitude - 0.2:
                channels.throttle = 1800
            else:
                self.set_mode(FlightMode.ALTHOLD)

        elif self._mode == FlightMode.LAND:
            if self._position.z > 0.1:
                channels.throttle = int(1500 - self._landing_speed * 300)
                channels.throttle = max(1100, channels.throttle)
            else:
                channels.throttle = 1000
                self.set_mode(FlightMode.MANUAL)

        channels.roll = max(1000, min(2000, channels.roll))
        channels.pitch = max(1000, min(2000, channels.pitch))
        channels.throttle = max(1000, min(2000, channels.throttle))

        return channels

    def get_status(self) -> dict:
        return {
            "mode": self._mode.name,
            "pos": (self._position.x, self._position.y, self._position.z),
            "target": (self._target.x, self._target.y, self._target.z),
            "wpt": f"{self._current_waypoint}/{len(self._waypoints)}",
        }


class _PID:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = time.time()

    def compute(self, setpoint: float, measurement: float) -> float:
        now = time.time()
        dt = max(now - self._prev_time, 0.001)
        self._prev_time = now

        error = setpoint - measurement
        self._integral = max(-100, min(100, self._integral + error * dt))
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        return self.kp * error + self.ki * self._integral + self.kd * derivative

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
