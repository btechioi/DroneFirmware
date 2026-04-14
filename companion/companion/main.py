"""Drone Companion Computer - Position estimation and corrections for Raspberry Pi Zero 2W.

Receives RC commands from ESP32 (via serial) or via SPI from Pico,
computes position corrections, sends back to Pico for motor control.
"""

from __future__ import annotations

import logging
import time
import argparse
from dataclasses import dataclass
from enum import Enum

logging.basicConfig(
    level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s"
)
logger = logging.getLogger(__name__)


class FlightMode(Enum):
    STABILIZE = 0
    ALTHOLD = 1
    POSHOLD = 2
    WAYPOINT = 3
    RTL = 4
    TAKEOFF = 5
    LAND = 6


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


@dataclass
class Telemetry:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    altitude: float = 0.0
    armed: bool = False
    mode: int = 0


class PID:
    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._i = 0.0
        self._prev = 0.0
        self._t = time.time()

    def compute(self, sp: float, pv: float) -> float:
        t = time.time()
        dt = max(t - self._t, 0.001)
        self._t = t

        e = sp - pv
        self._i = max(-50, min(50, self._i + e * dt))
        d = (e - self._prev) / dt
        self._prev = e

        return self.kp * e + self.ki * self._i + self.kd * d

    def reset(self) -> None:
        self._i = 0.0
        self._prev = 0.0


class Autopilot:
    def __init__(self) -> None:
        self._mode = FlightMode.ALTHOLD
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 1.5
        self._home_x = 0.0
        self._home_y = 0.0
        self._home_z = 0.0

        self._alt_pid = PID(1.5, 0.02, 0.5)
        self._px_pid = PID(2.0, 0.1, 0.5)
        self._py_pid = PID(2.0, 0.1, 0.5)

        self._waypoints = []
        self._wpt_idx = 0
        self._wpt_time = 0.0

    @property
    def mode(self) -> FlightMode:
        return self._mode

    def set_mode(self, mode: FlightMode) -> None:
        if mode != self._mode:
            logger.info(f"Mode: {self._mode.name} -> {mode.name}")
            self._mode = mode
            self._alt_pid.reset()
            self._px_pid.reset()
            self._py_pid.reset()

            if mode == FlightMode.TAKEOFF:
                self._target_z = 1.5
            elif mode == FlightMode.LAND:
                self._target_z = 0.0

    def set_position(self, x: float, y: float, z: float) -> None:
        self._pos_x = x
        self._pos_y = y

    def set_target(self, x: float, y: float, z: float) -> None:
        self._target_x = x
        self._target_y = y
        self._target_z = z

    def set_home(self) -> None:
        self._home_x = self._pos_x
        self._home_y = self._pos_y
        self._home_z = 0.0
        logger.info(f"Home: ({self._home_x:.1f}, {self._home_y:.1f})")

    def load_mission(self, waypoints: list) -> None:
        self._waypoints = waypoints
        self._wpt_idx = 0
        logger.info(f"Mission: {len(waypoints)} waypoints")

    def compute_corrections(
        self, rc: RCChannels, alt: float
    ) -> tuple[int, int, int, int]:
        roll = rc.roll
        pitch = rc.pitch
        throttle = rc.throttle
        yaw = rc.yaw

        if self._mode == FlightMode.ALTHOLD:
            throttle = int(1500 + self._alt_pid.compute(self._target_z, alt))

        elif self._mode == FlightMode.POSHOLD:
            dx = self._target_x - self._pos_x
            dy = self._target_y - self._pos_y
            pitch = int(1500 + self._px_pid.compute(dx, 0))
            roll = int(1500 + self._py_pid.compute(dy, 0))
            throttle = int(1500 + self._alt_pid.compute(self._target_z, alt))

        elif self._mode == FlightMode.RTL:
            dx = self._home_x - self._pos_x
            dy = self._home_y - self._pos_y
            pitch = int(1500 + self._px_pid.compute(dx, 0))
            roll = int(1500 + self._py_pid.compute(dy, 0))
            throttle = int(1500 + self._alt_pid.compute(2.0, alt))

        elif self._mode == FlightMode.WAYPOINT:
            if self._wpt_idx < len(self._waypoints):
                wp = self._waypoints[self._wpt_idx]
                dx = wp[0] - self._pos_x
                dy = wp[1] - self._pos_y
                dist = (dx**2 + dy**2) ** 0.5

                if dist < 0.5:
                    if time.time() - self._wpt_time > wp[3]:
                        self._wpt_idx += 1
                        if self._wpt_idx < len(self._waypoints):
                            logger.info(f"WPT {self._wpt_idx}")
                else:
                    pitch = int(1500 + (dx / max(dist, 0.1)) * 200)
                    roll = int(1500 + (dy / max(dist, 0.1)) * 200)

                if self._wpt_idx < len(self._waypoints):
                    throttle = int(
                        1500
                        + self._alt_pid.compute(self._waypoints[self._wpt_idx][2], alt)
                    )

        elif self._mode == FlightMode.TAKEOFF:
            if alt < 1.3:
                throttle = 1800
            else:
                self.set_mode(FlightMode.ALTHOLD)

        elif self._mode == FlightMode.LAND:
            if alt > 0.2:
                throttle = int(1500 - 100)
                throttle = max(1100, throttle)
            else:
                throttle = 1000
                self.set_mode(FlightMode.STABILIZE)

        return (
            max(1000, min(2000, roll)),
            max(1000, min(2000, pitch)),
            max(1000, min(2000, throttle)),
            max(1000, min(2000, yaw)),
        )

    def get_status(self) -> dict:
        return {
            "mode": self._mode.name,
            "pos": (self._pos_x, self._pos_y),
            "target": (self._target_x, self._target_y, self._target_z),
        }


class CompanionComputer:
    def __init__(self, args) -> None:
        self._running = False
        self._args = args

        self._serial = None
        self._spi = None
        self._autopilot = Autopilot()
        self._optical_flow = None

        self._flow_x = 0.0
        self._flow_y = 0.0
        self._altitude = 0.0
        self._armed = False
        self._rc = RCChannels()

        self._loop_hz = 50

    def initialize(self) -> bool:
        logger.info("Drone Companion v1.0 - Pi Zero 2W")

        self._init_serial()
        self._init_spi()
        self._init_optical_flow()

        self._autopilot.set_mode(FlightMode(self._args.mode))

        if self._args.waypoints:
            wps = []
            for wp in self._args.waypoints:
                parts = wp.split(",")
                if len(parts) >= 3:
                    wps.append((float(parts[0]), float(parts[1]), float(parts[2]), 0.0))
            self._autopilot.load_mission(wps)

        return True

    def _init_serial(self) -> None:
        if not self._args.serial:
            return
        try:
            import serial

            self._serial = serial.Serial(self._args.serial, 115200, timeout=0.1)
            logger.info(f"Serial connected: {self._args.serial}")
        except Exception as e:
            logger.error(f"Serial init failed: {e}")

    def _init_spi(self) -> None:
        try:
            from companion.comms.spi_link import SPIFlightLink

            self._spi = SPIFlightLink(
                bus=self._args.spi_bus,
                device=self._args.spi_device,
                speed_hz=self._args.spi_speed,
            )
            if self._spi.open():
                self._spi.start()
                logger.info("SPI active")
        except Exception as e:
            logger.warning(f"SPI unavailable: {e}")

    def _init_optical_flow(self) -> None:
        if not self._args.enable_optical_flow:
            return
        try:
            from companion.vision.optical_flow import PMW3901Sensor

            self._optical_flow = PMW3901Sensor()
            if self._optical_flow.open():
                logger.info("Optical flow active")
        except Exception as e:
            logger.warning(f"Optical flow failed: {e}")

    def run(self) -> None:
        self._running = True
        logger.info("Running...")

        loop_time = 1.0 / self._loop_hz
        last_heartbeat = 0

        while self._running:
            start = time.time()

            self._read_serial_rc()
            self._update_optical_flow()
            self._read_spi_telemetry()

            roll, pitch, throttle, yaw = self._autopilot.compute_corrections(
                self._rc, self._altitude
            )

            if self._spi:
                from companion.comms.spi_link import RCChannels, SPIPacketType, SPIFrame
                import struct

                if time.time() - last_heartbeat > 0.5:
                    heartbeat = SPIFrame.build(
                        SPIPacketType.HEARTBEAT,
                        struct.pack("<I", int(time.time() * 1000)),
                    )
                    self._spi.send_command(0x01, heartbeat)
                    last_heartbeat = time.time()

                out_rc = RCChannels(roll, pitch, throttle, yaw, self._rc.aux1)
                self._spi.send_rc_channels(out_rc)

            elapsed = time.time() - start
            if elapsed < loop_time:
                time.sleep(loop_time - elapsed)

        logger.info("Stopped")

    def _read_serial_rc(self) -> None:
        if not self._serial:
            return
        try:
            if self._serial.in_waiting >= 16:
                data = self._serial.read(16)
                if len(data) == 16:
                    self._rc.roll = int.from_bytes(data[0:2], "little")
                    self._rc.pitch = int.from_bytes(data[2:4], "little")
                    self._rc.throttle = int.from_bytes(data[4:6], "little")
                    self._rc.yaw = int.from_bytes(data[6:8], "little")
                    self._rc.aux1 = int.from_bytes(data[8:10], "little")

                    mode_byte = data[10]
                    if mode_byte < 7:
                        self._autopilot.set_mode(FlightMode(mode_byte))
        except Exception as e:
            logger.debug(f"Serial read: {e}")

    def _update_optical_flow(self) -> None:
        if not self._optical_flow:
            return
        flow = self._optical_flow.update()
        if flow:
            self._flow_x += flow.dx * 0.001
            self._flow_y += flow.dy * 0.001
            self._autopilot.set_position(self._flow_x, self._flow_y, self._altitude)

    def _read_spi_telemetry(self) -> None:
        if not self._spi:
            return
        tel = self._spi.get_last_telemetry()
        if tel:
            self._altitude = tel.altitude
            self._armed = tel.armed

    def stop(self) -> None:
        self._running = False
        if self._serial:
            self._serial.close()
        if self._spi:
            self._spi.close()


def main() -> None:
    import signal

    parser = argparse.ArgumentParser(description="Drone Companion")
    parser.add_argument(
        "--serial", type=str, default="/dev/serial0", help="Serial port for RC input"
    )
    parser.add_argument("--spi-bus", type=int, default=0)
    parser.add_argument("--spi-device", type=int, default=0)
    parser.add_argument("--spi-speed", type=int, default=20_000_000)
    parser.add_argument(
        "--mode",
        type=int,
        default=1,
        help="Flight mode: 0=stabilize, 1=althold, 2=poshold, 3=waypoint, 4=rtl, 5=takeoff, 6=land",
    )
    parser.add_argument("--waypoints", nargs="+", help="x,y,z waypoints")
    parser.add_argument("--enable-optical-flow", action="store_true")

    args = parser.parse_args()

    companion = CompanionComputer(args)

    def signal_handler(sig, frame):
        companion.stop()
        exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if companion.initialize():
        companion.run()


if __name__ == "__main__":
    main()
