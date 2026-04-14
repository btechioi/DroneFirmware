"""Mavlink Protocol Handler for Drone GCS"""

from __future__ import annotations

import struct
import logging
from dataclasses import dataclass, field
from datetime import datetime
from enum import IntEnum
from typing import Callable, Optional

try:
    from pymavlink import mavutil

    PymavlinkAvailable = True
except ImportError:
    PymavlinkAvailable = False
    mavutil = None

logger = logging.getLogger(__name__)


class TuneState(IntEnum):
    IDLE = 0
    WAITING_FOR_EXCITATION = 1
    COLLECTING_DATA = 2
    ANALYZING = 3
    APPLYING = 4
    COMPLETE = 5
    FAILED = 6


class TuneMethod(IntEnum):
    ZIEGLER_NICHOLS = 0
    RELAY = 1
    STEP_RESPONSE = 2
    FREQUENCY_SWEEP = 3
    MANUAL = 4


class TuneAxis(IntEnum):
    ROLL_RATE = 0
    PITCH_RATE = 1
    YAW_RATE = 2
    ROLL_ATTITUDE = 3
    PITCH_ATTITUDE = 4
    YAW_ATTITUDE = 5
    ALTITUDE = 6
    POSITION = 7


@dataclass
class MavlinkMessage:
    msg_id: int
    name: str
    timestamp: float
    data: dict = field(default_factory=dict)


@dataclass
class PIDGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    axis: int = 0

    def to_bytes(self) -> bytes:
        return struct.pack("<fffB", self.kp, self.ki, self.kd, self.axis)

    @classmethod
    def from_bytes(cls, data: bytes) -> PIDGains:
        kp, ki, kd, axis = struct.unpack("<fffB", data)
        return cls(kp=kp, ki=ki, kd=kd, axis=axis)


@dataclass
class PIDProfile:
    roll_rate_kp: float = 0.0
    roll_rate_ki: float = 0.0
    roll_rate_kd: float = 0.0
    pitch_rate_kp: float = 0.0
    pitch_rate_ki: float = 0.0
    pitch_rate_kd: float = 0.0
    yaw_rate_kp: float = 0.0
    yaw_rate_ki: float = 0.0
    yaw_rate_kd: float = 0.0
    roll_att_kp: float = 0.0
    roll_att_ki: float = 0.0
    roll_att_kd: float = 0.0
    pitch_att_kp: float = 0.0
    pitch_att_ki: float = 0.0
    pitch_att_kd: float = 0.0
    yaw_att_kp: float = 0.0
    yaw_att_ki: float = 0.0
    yaw_att_kd: float = 0.0
    alt_kp: float = 0.0
    alt_ki: float = 0.0
    alt_kd: float = 0.0
    pos_kp: float = 0.0
    pos_ki: float = 0.0
    pos_kd: float = 0.0
    version: int = 0
    timestamp: int = 0

    def to_bytes(self) -> bytes:
        return struct.pack(
            "<27f2I",
            self.roll_rate_kp,
            self.roll_rate_ki,
            self.roll_rate_kd,
            self.pitch_rate_kp,
            self.pitch_rate_ki,
            self.pitch_rate_kd,
            self.yaw_rate_kp,
            self.yaw_rate_ki,
            self.yaw_rate_kd,
            self.roll_att_kp,
            self.roll_att_ki,
            self.roll_att_kd,
            self.pitch_att_kp,
            self.pitch_att_ki,
            self.pitch_att_kd,
            self.yaw_att_kp,
            self.yaw_att_ki,
            self.yaw_att_kd,
            self.alt_kp,
            self.alt_ki,
            self.alt_kd,
            self.pos_kp,
            self.pos_ki,
            self.pos_kd,
            self.version,
            self.timestamp,
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> PIDProfile:
        values = struct.unpack("<27f2I", data)
        return cls(
            roll_rate_kp=values[0],
            roll_rate_ki=values[1],
            roll_rate_kd=values[2],
            pitch_rate_kp=values[3],
            pitch_rate_ki=values[4],
            pitch_rate_kd=values[5],
            yaw_rate_kp=values[6],
            yaw_rate_ki=values[7],
            yaw_rate_kd=values[8],
            roll_att_kp=values[9],
            roll_att_ki=values[10],
            roll_att_kd=values[11],
            pitch_att_kp=values[12],
            pitch_att_ki=values[13],
            pitch_att_kd=values[14],
            yaw_att_kp=values[15],
            yaw_att_ki=values[16],
            yaw_att_kd=values[17],
            alt_kp=values[18],
            alt_ki=values[19],
            alt_kd=values[20],
            pos_kp=values[21],
            pos_ki=values[22],
            pos_kd=values[23],
            version=values[24],
            timestamp=values[25],
        )

    @classmethod
    def default(cls) -> PIDProfile:
        return cls(
            roll_rate_kp=0.8,
            roll_rate_ki=0.02,
            roll_rate_kd=0.05,
            pitch_rate_kp=0.8,
            pitch_rate_ki=0.02,
            pitch_rate_kd=0.05,
            yaw_rate_kp=1.0,
            yaw_rate_ki=0.03,
            yaw_rate_kd=0.06,
            roll_att_kp=4.0,
            roll_att_ki=0.0,
            roll_att_kd=0.3,
            pitch_att_kp=4.0,
            pitch_att_ki=0.0,
            pitch_att_kd=0.3,
            yaw_att_kp=3.5,
            yaw_att_ki=0.0,
            yaw_att_kd=0.25,
            alt_kp=1.5,
            alt_ki=0.02,
            alt_kd=0.5,
            pos_kp=2.0,
            pos_ki=0.1,
            pos_kd=0.5,
        )


class MavlinkProtocol:
    CUSTOM_MSG_PID_TUNE = 200
    CUSTOM_MSG_PID_STATUS = 201
    CUSTOM_MSG_PID_GAINS = 202

    def __init__(self) -> None:
        self.mav: Optional[object] = None
        self.target_system: int = 1
        self.target_component: int = 1
        self._callbacks: dict = {}
        self._tune_callback: Optional[Callable] = None
        self._gains_callback: Optional[Callable] = None
        self._msg_count: dict = {}
        self._last_msg_time: dict = {}

    def connect_serial(self, port: str, baudrate: int = 115200) -> bool:
        if not PymavlinkAvailable:
            logger.error("pymavlink not available")
            return False
        try:
            self.mav = mavutil.mavserial_connection(
                port,
                baud=baudrate,
                autoreconnect=True,
                source_system=255,
                source_component=0,
            )
            self.mav.mav.srcComponent = 0
            self.mav.mav.srcSystem = 255
            logger.info(f"Connected to {port} at {baudrate} baud")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False

    def connect_udp(self, host: str = "0.0.0.0", port: int = 14550) -> bool:
        if not PymavlinkAvailable:
            return False
        try:
            self.mav = mavutil.mavudp(f"{host}:{port}", input=False, source_system=255)
            logger.info(f"UDP listener on {host}:{port}")
            return True
        except Exception as e:
            logger.error(f"UDP connection failed: {e}")
            return False

    def disconnect(self) -> None:
        if self.mav:
            self.mav.close()
            self.mav = None

    def is_connected(self) -> bool:
        return self.mav is not None

    def receive_message(self, timeout: float = 0.1) -> Optional[MavlinkMessage]:
        if not self.mav:
            return None
        try:
            msg = self.mav.recv_match(blocking=True, timeout=timeout)
            if msg:
                msg_id = getattr(msg, "msgid", 0)
                self._msg_count[msg_id] = self._msg_count.get(msg_id, 0) + 1
                self._last_msg_time[msg_id] = datetime.now().timestamp()
                data = {f: getattr(msg, f, None) for f in msg._fieldnames}
                return MavlinkMessage(
                    msg_id=msg_id,
                    name=msg.get_type(),
                    timestamp=datetime.now().timestamp(),
                    data=data,
                )
        except Exception as e:
            logger.debug(f"Receive error: {e}")
        return None

    def on_message(self, msg_id: int, callback: Callable) -> None:
        self._callbacks[msg_id] = callback

    def send_rc_channels(
        self,
        roll: int = 1500,
        pitch: int = 1500,
        throttle: int = 1000,
        yaw: int = 1500,
        aux1: int = 1000,
        aux2: int = 1000,
        aux3: int = 1000,
        aux4: int = 1000,
    ) -> None:
        if self.mav and PymavlinkAvailable:
            self.mav.mav.rc_channels_override_send(
                self.target_system,
                self.target_component,
                roll,
                pitch,
                throttle,
                yaw,
                aux1,
                aux2,
                aux3,
                aux4,
            )

    def send_pid_tune_start(
        self, method: TuneMethod, axis: TuneAxis, amplitude: float = 50.0
    ) -> None:
        if self.mav and PymavlinkAvailable:
            data = struct.pack("<BBf", int(method), int(axis), amplitude)
            self.mav.mav.payload16_send(bytes([self.CUSTOM_MSG_PID_TUNE]) + data)

    def send_pid_tune_stop(self) -> None:
        if self.mav and PymavlinkAvailable:
            data = struct.pack("<B", 2)
            self.mav.mav.payload16_send(bytes([self.CUSTOM_MSG_PID_TUNE]) + data)

    def send_pid_gains(self, gains: PIDGains) -> None:
        if self.mav and PymavlinkAvailable:
            self.mav.mav.payload16_send(
                bytes([self.CUSTOM_MSG_PID_GAINS]) + gains.to_bytes()
            )

    def send_pid_profile(self, profile: PIDProfile) -> None:
        if self.mav and PymavlinkAvailable:
            self.mav.mav.payload16_send(
                bytes([self.CUSTOM_MSG_PID_GAINS]) + profile.to_bytes()
            )

    def request_all_gains(self) -> None:
        if self.mav and PymavlinkAvailable:
            data = struct.pack("<B", 0x21)
            self.mav.mav.payload16_send(bytes([self.CUSTOM_MSG_PID_TUNE]) + data)

    def save_pid_profile(self, slot: int) -> None:
        if self.mav and PymavlinkAvailable:
            data = struct.pack("<BB", 0x10, slot)
            self.mav.mav.payload16_send(bytes([self.CUSTOM_MSG_PID_TUNE]) + data)

    def load_pid_profile(self, slot: int) -> None:
        if self.mav and PymavlinkAvailable:
            data = struct.pack("<BB", 0x11, slot)
            self.mav.mav.payload16_send(bytes([self.CUSTOM_MSG_PID_TUNE]) + data)

    def get_message_rate(self, msg_id: int) -> Optional[int]:
        return self._msg_count.get(msg_id)

    def get_stats(self) -> dict:
        return {"msg_count": self._msg_count.copy(), "connected": self.is_connected()}
