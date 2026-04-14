"""Connection manager for Drone GCS - handles USB, Serial, and Relay connections"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable

try:
    import serial.tools.list_ports

    SerialAvailable = True
except ImportError:
    SerialAvailable = False
    serial = None

from ..protocol import MavlinkProtocol

logger = logging.getLogger(__name__)


USB_PID_VID_PAIRS = [
    ("2E3A", "0001"),  # Drone Flight Controller (custom VID/PID)
    ("2E3A", "1002"),  # Raspberry Pi Pico
    ("2E3A", "1003"),  # Raspberry Pi Pico (bootloader)
    ("2341", "0001"),  # Arduino
    ("2341", "0043"),  # Arduino Due
    ("0483", "3748"),  # STM32
    ("0403", "6001"),  # FTDI
    ("10c4", "ea60"),  # CP2102
    ("1d50", "60aa"),  # Black Pill
]

DRONE_VID_PID = ("2E3A", "0001")

DRONE_DEVICE_NAMES = [
    "pico",
    "drone",
    "fc",
    "flight",
    "droneflightcontroller",
    "drone firmware",
    "droneflight",
    "rp2040",
    "raspberry",
    "arduino",
    "controller",
    "rp2040",
    "raspberry",
    "arduino",
]


class ConnectionType(Enum):
    NONE = "none"
    USB_SERIAL = "usb_serial"
    UDP = "udp"
    TCP = "tcp"
    RELAY_USB = "relay_usb"


@dataclass
class ConnectionInfo:
    type: ConnectionType
    port: str
    baudrate: int = 115200
    host: str = ""
    port_num: int = 0


@dataclass
class USBDevice:
    port: str
    description: str
    vid: str
    pid: str
    serial: str
    is_drone: bool


class ConnectionManager:
    def __init__(self) -> None:
        self.protocol = MavlinkProtocol()
        self._connection_type = ConnectionType.NONE
        self._connected = False
        self._receiver_thread: Optional[threading.Thread] = None
        self._running = False
        self._message_callback: Optional[Callable] = None
        self._status_callback: Optional[Callable] = None
        self._detected_drone_port: Optional[str] = None

    @property
    def is_connected(self) -> bool:
        return self._connected and self.protocol.is_connected()

    @property
    def connection_type(self) -> ConnectionType:
        return self._connection_type

    def set_message_callback(self, callback: Callable) -> None:
        self._message_callback = callback

    def set_status_callback(self, callback: Callable) -> None:
        self._status_callback = callback

    def detect_drone_device(self) -> Optional[USBDevice]:
        if not SerialAvailable:
            return None

        ports = serial.tools.list_ports.comports()

        # First check for our custom Drone Flight Controller VID/PID
        for port_info in ports:
            vid = f"{port_info.vid:04X}" if port_info.vid else ""
            pid = f"{port_info.pid:04X}" if port_info.pid else ""
            desc = (port_info.description or "").lower()
            serial_num = port_info.serial_number or ""

            # Check for custom Drone Flight Controller (priority)
            if vid == DRONE_VID_PID[0] and pid == DRONE_VID_PID[1]:
                device = USBDevice(
                    port=port_info.device,
                    description=f"Drone Flight Controller ({port_info.description or 'Custom USB'})",
                    vid=vid,
                    pid=pid,
                    serial=serial_num,
                    is_drone=True,
                )
                logger.info(f"🎯 Detected Drone Flight Controller: {device.port}")
                return device

        # Then check other known flight controller VID/PIDs
        for port_info in ports:
            vid = f"{port_info.vid:04X}" if port_info.vid else ""
            pid = f"{port_info.pid:04X}" if port_info.pid else ""
            desc = (port_info.description or "").lower()
            serial_num = port_info.serial_number or ""

            for v, p in USB_PID_VID_PAIRS:
                if vid == v and pid == p:
                    is_drone = any(name in desc for name in DRONE_DEVICE_NAMES) or any(
                        name in serial_num.lower() for name in DRONE_DEVICE_NAMES
                    )

                    device = USBDevice(
                        port=port_info.device,
                        description=port_info.description or "Unknown",
                        vid=vid,
                        pid=pid,
                        serial=serial_num,
                        is_drone=is_drone,
                    )
                    logger.info(
                        f"Detected device: {device.port} - {device.description}"
                    )
                    return device

        # Then check by device name
        for port_info in ports:
            desc = (port_info.description or "").lower()
            serial_num = port_info.serial_number or ""
            vid = f"{port_info.vid:04X}" if port_info.vid else ""
            pid = f"{port_info.pid:04X}" if port_info.pid else ""

            if any(name in desc for name in DRONE_DEVICE_NAMES) or any(
                name in serial_num.lower() for name in DRONE_DEVICE_NAMES
            ):
                device = USBDevice(
                    port=port_info.device,
                    description=port_info.description or "Unknown",
                    vid=vid,
                    pid=pid,
                    serial=serial_num,
                    is_drone=True,
                )
                logger.info(
                    f"Detected drone by name: {device.port} - {device.description}"
                )
                return device

        # Return first available port as fallback
        if ports:
            first_port = ports[0]
            device = USBDevice(
                port=first_port.device,
                description=first_port.description or "USB Device",
                vid=f"{first_port.vid:04X}" if first_port.vid else "",
                pid=f"{first_port.pid:04X}" if first_port.pid else "",
                serial=first_port.serial_number or "",
                is_drone=False,
            )
            logger.info(f"Auto-selected first available: {device.port}")
            return device

        return None

    def list_serial_ports(self) -> list[str]:
        if not SerialAvailable:
            return []
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def list_usb_devices(self) -> list[USBDevice]:
        devices = []
        if not SerialAvailable:
            return devices

        for port_info in serial.tools.list_ports.comports():
            vid = f"{port_info.vid:04X}" if port_info.vid else ""
            pid = f"{port_info.pid:04X}" if port_info.pid else ""
            devices.append(
                USBDevice(
                    port=port_info.device,
                    description=port_info.description or "Unknown",
                    vid=vid,
                    pid=pid,
                    serial=port_info.serial_number or "",
                    is_drone=any(
                        name in (port_info.description or "").lower()
                        for name in DRONE_DEVICE_NAMES
                    )
                    or (vid == DRONE_VID_PID[0] and pid == DRONE_VID_PID[1]),
                )
            )
        return devices

    def auto_connect(self, baudrate: int = 115200) -> bool:
        device = self.detect_drone_device()
        if device:
            self._detected_drone_port = device.port
            return self.connect_usb(device.port, baudrate)
        return False

    def connect_usb(self, port: str, baudrate: int = 115200) -> bool:
        self.disconnect()
        logger.info(f"Connecting to USB serial: {port} @ {baudrate}")
        if self.protocol.connect_serial(port, baudrate):
            self._connection_type = ConnectionType.USB_SERIAL
            self._connected = True
            self._start_receiver()
            return True
        return False

    def connect_udp(self, host: str = "0.0.0.0", port: int = 14550) -> bool:
        self.disconnect()
        logger.info(f"Connecting to UDP: {host}:{port}")
        if self.protocol.connect_udp(host, port):
            self._connection_type = ConnectionType.UDP
            self._connected = True
            self._start_receiver()
            return True
        return False

    def connect_relay(
        self, relay_port: str, drone_port: str, baudrate: int = 115200
    ) -> bool:
        self.disconnect()
        logger.info(f"Connecting relay: {relay_port} -> {drone_port}")
        if self.protocol.connect_serial(relay_port, baudrate):
            self._connection_type = ConnectionType.RELAY_USB
            self._connected = True
            self._start_receiver()
            return True
        return False

    def disconnect(self) -> None:
        self._running = False
        if self._receiver_thread and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=2.0)
        self.protocol.disconnect()
        self._connected = False
        self._connection_type = ConnectionType.NONE
        logger.info("Disconnected")

    def _start_receiver(self) -> None:
        self._running = True
        self._receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receiver_thread.start()

    def _receive_loop(self) -> None:
        while self._running and self.is_connected:
            msg = self.protocol.receive_message(timeout=0.1)
            if msg and self._message_callback:
                self._message_callback(msg)
            if self._status_callback:
                self._status_callback(self.is_connected)

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
        self.protocol.send_rc_channels(
            roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4
        )

    def send_pid_tune_start(
        self, method: int, axis: int, amplitude: float = 50.0
    ) -> None:
        from ..protocol import TuneMethod, TuneAxis

        self.protocol.send_pid_tune_start(TuneMethod(method), TuneAxis(axis), amplitude)

    def send_pid_tune_stop(self) -> None:
        self.protocol.send_pid_tune_stop()

    def send_pid_gains(self, axis: int, kp: float, ki: float, kd: float) -> None:
        from ..protocol import PIDGains

        self.protocol.send_pid_gains(PIDGains(kp=kp, ki=ki, kd=kd, axis=axis))

    def send_pid_profile(self, profile) -> None:
        self.protocol.send_pid_profile(profile)

    def request_all_gains(self) -> None:
        self.protocol.request_all_gains()

    def save_pid_profile(self, slot: int) -> None:
        self.protocol.save_pid_profile(slot)

    def load_pid_profile(self, slot: int) -> None:
        self.protocol.load_pid_profile(slot)

    def get_stats(self) -> dict:
        return self.protocol.get_stats()
