"""
DroneFlightController Test Suite

Comprehensive testing for the drone flight controller firmware.
Tests PID controllers, motor mixing, failsafes, and RC handling.
"""

import struct
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional, Tuple
import numpy as np


class FlightMode(IntEnum):
    MANUAL = 0
    ALTHOLD = 1
    POSHOLD = 2
    WAYPOINT = 3
    RTL = 4
    TAKEOFF = 5
    LAND = 6


class FailSafeState(IntEnum):
    NONE = 0
    SIGNAL_LOSS = 1
    LOW_BATTERY = 2
    CRITICAL_SENSOR = 3


class ControlSource(IntEnum):
    RC_RECEIVER = 0
    COMPANION = 1
    FAILSAFE = 2


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

    def to_bytes(self) -> bytes:
        data = struct.pack(
            "<HHHHHHHH",
            self.roll,
            self.pitch,
            self.throttle,
            self.yaw,
            self.aux1,
            self.aux2,
            self.aux3,
            self.aux4,
        )
        crc = sum(data) & 0xFFFF
        return bytes([0xAA, 0x01, 8]) + data + struct.pack("<H", crc)

    @classmethod
    def from_bytes(cls, data: bytes) -> "RCChannels":
        if len(data) < 19:
            raise ValueError(f"RC packet too short: {len(data)}")
        roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4 = struct.unpack(
            "<HHHHHHHH", data[3:19]
        )
        return cls(roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4)


@dataclass
class Telemetry:
    armed: bool = False
    mode: int = 0
    failsafe: int = 0
    control_source: int = 0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    altitude: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    battery_mv: int = 12000

    def to_bytes(self) -> bytes:
        flags = (1 if self.armed else 0) | (1 if self.control_source == 1 else 0) << 1
        flags |= self.failsafe << 4

        data = struct.pack(
            "<Bbbbii",
            flags,
            self.control_source,
            int(self.roll * 100),
            int(self.pitch * 100),
            int(self.yaw * 100),
            int(self.altitude * 100),
        )
        data += struct.pack(
            "<hhh",
            int(self.gyro_x * 100),
            int(self.gyro_y * 100),
            int(self.gyro_z * 100),
        )
        data += struct.pack("<H", self.battery_mv)
        crc = sum(data) & 0xFFFF
        return bytes([0x55, 0x02, 0]) + data + struct.pack("<H", crc)

    @classmethod
    def from_bytes(cls, data: bytes) -> "Telemetry":
        if len(data) < 24:
            raise ValueError(f"Telemetry too short: {len(data)}")
        t = cls()
        flags = data[0]
        t.armed = bool(flags & 1)
        t.control_source = (flags >> 1) & 1
        t.failsafe = (flags >> 4) & 0x0F
        t.roll = struct.unpack("<h", data[2:4])[0] / 100.0
        t.pitch = struct.unpack("<h", data[4:6])[0] / 100.0
        t.yaw = struct.unpack("<h", data[6:8])[0] / 100.0
        t.altitude = struct.unpack("<i", data[8:12])[0] / 100.0
        t.gyro_x = struct.unpack("<h", data[12:14])[0] / 100.0
        t.gyro_y = struct.unpack("<h", data[14:16])[0] / 100.0
        t.gyro_z = struct.unpack("<h", data[16:18])[0] / 100.0
        t.battery_mv = struct.unpack("<H", data[18:20])[0]
        return t


class PID:
    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limit: float = 500.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, error: float, dt: Optional[float] = None) -> float:
        if dt is None:
            now = time.time()
            if self._prev_time is None:
                dt = 0.01
            else:
                dt = now - self._prev_time
            self._prev_time = now

        dt = max(dt, 0.001)

        self._integral = max(-50, min(50, self._integral + error * dt))
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None


class MotorMixer:
    def __init__(self):
        self.motor_count = 4
        self.motor_outputs = [1000, 1000, 1000, 1000]

    def compute(
        self, throttle: float, roll: float, pitch: float, yaw: float
    ) -> List[int]:
        throttle = max(1000, min(2000, throttle))

        m0 = throttle + roll + pitch + yaw
        m1 = throttle - roll + pitch - yaw
        m2 = throttle - roll - pitch + yaw
        m3 = throttle + roll - pitch - yaw

        outputs = [m0, m1, m2, m3]
        for i in range(self.motor_count):
            self.motor_outputs[i] = max(1000, min(2000, outputs[i]))

        return self.motor_outputs


class IMUSimulator:
    def __init__(self):
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._yaw_rate = 0.0
        self._altitude = 0.0

    def update(self, dt: float):
        self._roll += self._roll_rate * dt
        self._pitch += self._pitch_rate * dt
        self._yaw += self._yaw_rate * dt
        self._yaw = self._yaw % (2 * np.pi)

    def apply_control(
        self, roll_rate_cmd: float, pitch_rate_cmd: float, yaw_rate_cmd: float
    ):
        self._roll_rate = roll_rate_cmd
        self._pitch_rate = pitch_rate_cmd
        self._yaw_rate = yaw_rate_cmd

    @property
    def roll(self) -> float:
        return self._roll

    @property
    def pitch(self) -> float:
        return self._pitch

    @property
    def yaw(self) -> float:
        return self._yaw

    @property
    def roll_rate(self) -> float:
        return self._roll_rate

    @property
    def pitch_rate(self) -> float:
        return self._pitch_rate

    @property
    def yaw_rate(self) -> float:
        return self._yaw_rate

    @property
    def altitude(self) -> float:
        return self._altitude


class FailsafeManager:
    def __init__(self):
        self.state = FailSafeState.NONE
        self.rc_timeout_ms = 500
        self.stuck_timeout_ms = 3000
        self.stuck_threshold = 20
        self._last_rc_time = 0
        self._last_control_change = 0
        self._prev_roll = 1500
        self._prev_pitch = 1500
        self._prev_yaw = 1500
        self.rc_lost = False

    def update(self, now_ms: int, rc: RCChannels):
        if now_ms - self._last_rc_time > self.rc_timeout_ms and not self.rc_lost:
            self.rc_lost = True
            if self.state == FailSafeState.NONE:
                self.state = FailSafeState.SIGNAL_LOSS

        roll_changed = abs(rc.roll - self._prev_roll) > self.stuck_threshold
        pitch_changed = abs(rc.pitch - self._prev_pitch) > self.stuck_threshold
        yaw_changed = abs(rc.yaw - self._prev_yaw) > self.stuck_threshold

        if roll_changed or pitch_changed or yaw_changed:
            self._last_control_change = now_ms
            self._prev_roll = rc.roll
            self._prev_pitch = rc.pitch
            self._prev_yaw = rc.yaw

        if now_ms - self._last_rc_time < self.rc_timeout_ms:
            self.rc_lost = False
            self._last_rc_time = now_ms

    def signal_rc_received(self, now_ms: int):
        self._last_rc_time = now_ms

    def reset(self):
        self.state = FailSafeState.NONE
        self.rc_lost = False


class FlightControllerSimulator:
    def __init__(self):
        self.rc = RCChannels()
        self.telemetry = Telemetry()
        self.imu = IMUSimulator()
        self.mixer = MotorMixer()
        self.failsafe = FailsafeManager()

        self.motors_armed = False
        self.current_mode = FlightMode.MANUAL
        self.active_control = ControlSource.RC_RECEIVER
        self.companion_connected = False

        self.pid_roll = PID(0.5, 0.0, 0.05, 500)
        self.pid_pitch = PID(0.5, 0.0, 0.05, 500)
        self.pid_yaw = PID(0.5, 0.0, 0.05, 500)

        self._last_loop_time = time.time()
        self._loop_count = 0
        self._last_rc_receive = 0

    def arm(self):
        if self.failsafe.state != FailSafeState.CRITICAL_SENSOR:
            self.motors_armed = True
            return True
        return False

    def disarm(self):
        self.motors_armed = False
        self.failsafe.reset()
        for i in range(4):
            self.mixer.motor_outputs[i] = 1000
        return True

    def receive_rc(self, rc: RCChannels, now_ms: int):
        self.rc = rc
        self._last_rc_receive = now_ms
        self.failsafe.signal_rc_received(now_ms)

        if rc.aux1 > 1500 and not self.motors_armed:
            self.arm()
        elif rc.aux1 < 1200 and self.motors_armed:
            self.disarm()

        if rc.aux2 > 1500:
            self.current_mode = FlightMode.ALTHOLD
        else:
            self.current_mode = FlightMode.MANUAL

    def update(self, dt: float):
        now_ms = int(time.time() * 1000)
        self.failsafe.update(now_ms, self.rc)

        if self.active_control == ControlSource.FAILSAFE:
            throttle = 1200
        else:
            throttle = self.rc.throttle

        if not self.motors_armed:
            throttle = 1000

        target_roll = (self.rc.roll - 1500) / 500.0
        target_pitch = (self.rc.pitch - 1500) / 500.0
        target_yaw = (self.rc.yaw - 1500) / 500.0

        roll_error = target_roll - self.imu.roll
        pitch_error = target_pitch - self.imu.pitch

        roll_corr = self.pid_roll.compute(roll_error, dt)
        pitch_corr = self.pid_pitch.compute(pitch_error, dt)
        yaw_corr = self.pid_yaw.compute(target_yaw - self.imu.yaw, dt)

        self.mixer.compute(throttle, roll_corr, pitch_corr, yaw_corr)

        self.imu.apply_control(roll_corr, pitch_corr, yaw_corr)
        self.imu.update(dt)

        self.telemetry.armed = self.motors_armed
        self.telemetry.mode = self.current_mode
        self.telemetry.failsafe = self.failsafe.state
        self.telemetry.control_source = self.active_control
        self.telemetry.roll = self.imu.roll
        self.telemetry.pitch = self.imu.pitch
        self.telemetry.yaw = self.imu.yaw
        self.telemetry.altitude = self.imu.altitude
        self.telemetry.gyro_x = self.imu.roll_rate
        self.telemetry.gyro_y = self.imu.pitch_rate
        self.telemetry.gyro_z = self.imu.yaw_rate

        self._loop_count += 1

    def get_motor_outputs(self) -> List[int]:
        return self.mixer.motor_outputs.copy()


class RCStatus:
    def __init__(self):
        self.uptime_ms: int = 0
        self.battery_mv: int = 12000
        self.battery_percent: int = 100
        self.rssi: int = -45
        self.signal_quality: int = 95
        self.packet_loss_x10: int = 0
        self.latency_us: int = 5000
        self.temperature_x10: int = 235
        self.free_memory: int = 40000
        self.cpu_load: int = 15
        self.espnow_channel: int = 6
        self.connection_state: int = 3
        self.error_flags: int = 0

    def to_bytes(self) -> bytes:
        data = struct.pack("<I", self.uptime_ms)
        data += struct.pack("<H", self.battery_mv)
        data += struct.pack("<Bb", self.battery_percent, self.rssi)
        data += struct.pack("<BH", self.signal_quality, self.packet_loss_x10)
        data += struct.pack("<I", self.latency_us)
        data += struct.pack("<h", self.temperature_x10)
        data += struct.pack("<I", self.free_memory)
        data += struct.pack(
            "<BBbB",
            self.cpu_load,
            self.espnow_channel,
            self.connection_state,
            self.error_flags,
        )
        data += bytes(2)
        crc = sum(data) & 0xFFFF
        return bytes([0x66, 0x03]) + data + struct.pack("<H", crc)


def run_simulation(
    fc: FlightControllerSimulator,
    duration_s: float,
    rc_sequence: List[Tuple[float, RCChannels]],
) -> dict:
    results = {
        "loop_count": 0,
        "telemetry_samples": [],
        "motor_samples": [],
        "armed": False,
        "failsafe_triggered": False,
    }

    dt = 0.0025
    steps = int(duration_s / dt)
    rc_idx = 0

    for step in range(steps):
        t = step * dt

        while rc_idx < len(rc_sequence) and rc_sequence[rc_idx][0] <= t:
            _, rc = rc_sequence[rc_idx]
            fc.receive_rc(rc, int(t * 1000))
            rc_idx += 1

        fc.update(dt)

        if not results["armed"] and fc.motors_armed:
            results["armed"] = True

        if (
            not results["failsafe_triggered"]
            and fc.failsafe.state != FailSafeState.NONE
        ):
            results["failsafe_triggered"] = True

        if step % 40 == 0:
            results["telemetry_samples"].append(fc.telemetry)
        if step % 40 == 0:
            results["motor_samples"].append(fc.get_motor_outputs())

        results["loop_count"] = step

    return results
