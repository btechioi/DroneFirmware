"""High-speed SPI communication link between companion computer and drone."""

from __future__ import annotations

import struct
import logging
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, Callable
from collections import deque

logger = logging.getLogger(__name__)

try:
    import spidev

    SPI_AVAILABLE = True
except ImportError:
    SPI_AVAILABLE = False
    spidev = None


class SPIPacketType(IntEnum):
    HEARTBEAT = 0x01
    TELEMETRY = 0x02
    COMMAND = 0x03
    CONFIG = 0x04
    RAW_SENSOR = 0x05
    RC_CHANNELS = 0x10
    MOTOR_OUTPUT = 0x11
    OPTICAL_FLOW = 0x12
    POSITION_UPDATE = 0x13
    ERROR_STATUS = 0x20
    NOP = 0xFF


SPI_SYNC_BYTE = 0xAA
SPI_TRAILER_BYTE = 0x55
SPI_MAX_SPEED_HZ = 125_000_000
SPI_DEFAULT_SPEED_HZ = 31_000_000
SPI_BITS_PER_WORD = 8


@dataclass
class TelemetryData:
    timestamp: int
    armed: bool
    mode: int
    roll: float
    pitch: float
    yaw: float
    altitude: float
    battery_voltage: float
    battery_current: float
    gps_lat: float
    gps_lon: float
    gps_sats: int
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0


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
        return struct.pack(
            "<8H",
            self.roll,
            self.pitch,
            self.throttle,
            self.yaw,
            self.aux1,
            self.aux2,
            self.aux3,
            self.aux4,
        )


@dataclass
class PositionUpdate:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    quality: int = 0

    def to_bytes(self) -> bytes:
        return struct.pack(
            "<6fi", self.x, self.y, self.z, self.vx, self.vy, self.vz, self.quality
        )


class SPICRC:
    POLYNOMIAL = 0xA001
    INITIAL = 0xFFFF

    @staticmethod
    def calculate(data: bytes) -> int:
        crc = SPICRC.INITIAL
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ SPICRC.POLYNOMIAL
                else:
                    crc >>= 1
        return crc


class SPIFrame:
    SYNC = SPI_SYNC_BYTE
    TRAILER = SPI_TRAILER_BYTE

    @staticmethod
    def build(packet_type: int, data: bytes) -> bytes:
        length = 3 + len(data)
        crc_data = bytes([packet_type]) + data
        crc = SPICRC.calculate(crc_data)

        frame = bytes([SPIFrame.SYNC, packet_type, length])
        frame += struct.pack("<H", crc)
        frame += data
        frame += bytes([SPIFrame.TRAILER])

        return frame

    @staticmethod
    def parse(frame: bytes) -> Optional[tuple[int, bytes]]:
        if len(frame) < 5:
            return None

        if frame[0] != SPIFrame.SYNC:
            return None

        packet_type = frame[1]
        length = frame[2]

        if len(frame) < length:
            return None

        if frame[length - 1] != SPIFrame.TRAILER:
            return None

        crc = struct.unpack("<H", frame[3:5])[0]
        data = frame[5 : length - 1]

        calc_crc = SPICRC.calculate(bytes([packet_type]) + data)
        if crc != calc_crc:
            logger.warning(f"SPI CRC mismatch: got {crc:04x}, expected {calc_crc:04x}")
            return None

        return packet_type, data


class SPIFlightLink:
    def __init__(
        self,
        bus: int = 0,
        device: int = 0,
        speed_hz: int = SPI_DEFAULT_SPEED_HZ,
        mode: int = 0,
    ) -> None:
        if not SPI_AVAILABLE:
            raise RuntimeError("spidev not available")

        self.bus = bus
        self.device = device
        self.speed_hz = min(speed_hz, SPI_MAX_SPEED_HZ)
        self.mode = mode

        self._spi: Optional[spidev.SpiDev] = None
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None

        self._telemetry_callback: Optional[Callable[[TelemetryData], None]] = None
        self._command_callback: Optional[Callable[[bytes], None]] = None

        self._telemetry_queue: deque = deque(maxlen=100)
        self._last_telemetry: Optional[TelemetryData] = None
        self._last_heartbeat = 0

        self._stats = {"tx_frames": 0, "rx_frames": 0, "crc_errors": 0, "last_speed": 0}

    def open(self) -> bool:
        try:
            self._spi = spidev.SpiDev()
            self._spi.open(self.bus, self.device)
            self._spi.max_speed_hz = self.speed_hz
            self._spi.mode = self.mode
            self._spi.bits_per_word = SPI_BITS_PER_WORD

            logger.info(
                f"SPI link opened: bus={self.bus}, device={self.device}, "
                f"speed={self.speed_hz / 1_000_000:.1f}MHz"
            )
            return True
        except Exception as e:
            logger.error(f"Failed to open SPI link: {e}")
            return False

    def close(self) -> None:
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._spi:
            self._spi.close()
            self._spi = None
        logger.info("SPI link closed")

    def set_telemetry_callback(self, callback: Callable[[TelemetryData], None]) -> None:
        self._telemetry_callback = callback

    def set_command_callback(self, callback: Callable[[bytes], None]) -> None:
        self._command_callback = callback

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        logger.info("SPI RX thread started")

    def _rx_loop(self) -> None:
        heartbeat_tx = SPIFrame.build(SPIPacketType.HEARTBEAT, struct.pack("<IB", 0, 0))

        while self._running:
            try:
                received = self._spi.xfer2(heartbeat_tx)

                if received and len(received) > 0:
                    self._process_received(bytes(received))

                self._stats["tx_frames"] += 1

                time.sleep(0.0001)

            except Exception as e:
                logger.error(f"SPI RX error: {e}")
                time.sleep(0.01)

    def _process_received(self, data: bytes) -> None:
        if len(data) < 5:
            return

        if data[0] != SPI_SYNC_BYTE:
            return

        frame_len = data[2]
        if len(data) < frame_len:
            return

        packet_type = data[1]
        payload = data[5 : frame_len - 1]

        if packet_type == SPIPacketType.TELEMETRY:
            self._parse_telemetry(payload)
        elif packet_type == SPIPacketType.HEARTBEAT:
            self._last_heartbeat = time.time()
        elif packet_type == SPIPacketType.ERROR_STATUS:
            logger.warning(f"Drone error status: {payload.hex()}")

        self._stats["rx_frames"] += 1

    def _parse_telemetry(self, data: bytes) -> None:
        if len(data) < 28:
            return

        try:
            timestamp, flags, roll, pitch, yaw, altitude, voltage, current = (
                struct.unpack("<IBfffffH", data[:28])
            )

            armed = bool(flags & 0x01)
            mode = (flags >> 1) & 0x0F

            telemetry = TelemetryData(
                timestamp=timestamp,
                armed=armed,
                mode=mode,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                altitude=altitude,
                battery_voltage=voltage,
                battery_current=current / 100.0,
                gps_lat=0.0,
                gps_lon=0.0,
                gps_sats=0,
            )

            self._last_telemetry = telemetry
            self._telemetry_queue.append(telemetry)

            if self._telemetry_callback:
                self._telemetry_callback(telemetry)

        except Exception as e:
            logger.error(f"Telemetry parse error: {e}")

    def send_rc_channels(self, channels: RCChannels) -> None:
        data = channels.to_bytes()
        frame = SPIFrame.build(SPIPacketType.RC_CHANNELS, data)

        try:
            self._spi.xfer2(list(frame))
            self._stats["tx_frames"] += 1
        except Exception as e:
            logger.error(f"Failed to send RC channels: {e}")

    def send_position_update(self, position: PositionUpdate) -> None:
        data = position.to_bytes()
        frame = SPIFrame.build(SPIPacketType.POSITION_UPDATE, data)

        try:
            self._spi.xfer2(list(frame))
        except Exception as e:
            logger.error(f"Failed to send position update: {e}")

    def send_command(self, command_id: int, data: bytes = b"") -> None:
        payload = bytes([command_id]) + data
        frame = SPIFrame.build(SPIPacketType.COMMAND, payload)

        try:
            self._spi.xfer2(list(frame))
        except Exception as e:
            logger.error(f"Failed to send command: {e}")

    def is_connected(self) -> bool:
        if time.time() - self._last_heartbeat > 3.0:
            return False
        return self._spi is not None

    def get_last_telemetry(self) -> Optional[TelemetryData]:
        return self._last_telemetry

    def get_stats(self) -> dict:
        return self._stats.copy()


class SPIBridge:
    def __init__(self, spi_link: SPIFlightLink) -> None:
        self._spi = spi_link
        self._serial_port: Optional[str] = None
        self._baudrate = 115200

    def bridge_spi_to_serial(self, serial_port: str, baudrate: int = 115200) -> None:
        import serial

        self._serial_port = serial_port
        self._baudrate = baudrate

        ser = serial.Serial(serial_port, baudrate)

        def bridge_loop():
            while True:
                if self._spi.is_connected():
                    telemetry = self._spi.get_last_telemetry()
                    if telemetry:
                        pass

        threading.Thread(target=bridge_loop, daemon=True).start()
