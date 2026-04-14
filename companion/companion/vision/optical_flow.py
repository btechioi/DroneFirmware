"""Optical flow position estimation for companion computer."""

from __future__ import annotations

import logging
import time
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)

try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    cv2 = None


@dataclass
class FlowData:
    dx: float
    dy: float
    confidence: float
    timestamp: float
    altitude: float = 1.0


@dataclass
class PositionEstimate:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    quality: int
    timestamp: float


class OpticalFlowSensor:
    def __init__(
        self,
        camera_id: int = 0,
        resolution: Tuple[int, int] = (320, 240),
        fps: int = 60,
    ) -> None:
        if not CV2_AVAILABLE:
            raise RuntimeError("OpenCV not available")

        self.camera_id = camera_id
        self.resolution = resolution
        self.fps = fps

        self._capture = None
        self._prev_gray = None
        self._prev_timestamp = 0
        self._position = np.array([0.0, 0.0, 0.0])
        self._velocity = np.array([0.0, 0.0, 0.0])
        self._altitude = 1.0

        self._lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )

        self._feature_params = dict(
            maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7
        )

        self._enabled = False
        self._quality_threshold = 0.5

    def open(self) -> bool:
        try:
            self._capture = cv2.VideoCapture(self.camera_id)
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self._capture.set(cv2.CAP_PROP_FPS, self.fps)

            if not self._capture.isOpened():
                logger.error("Failed to open camera")
                return False

            self._enabled = True
            logger.info(
                f"Camera opened: {self.resolution[0]}x{self.resolution[1]} @ {self.fps}fps"
            )
            return True
        except Exception as e:
            logger.error(f"Camera open failed: {e}")
            return False

    def close(self) -> None:
        self._enabled = False
        if self._capture:
            self._capture.release()
            self._capture = None

    def set_altitude(self, altitude: float) -> None:
        self._altitude = max(0.1, altitude)

    def update(self) -> Optional[FlowData]:
        if not self._enabled or not self._capture:
            return None

        ret, frame = self._capture.read()
        if not ret:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        timestamp = time.time()

        if self._prev_gray is None:
            self._prev_gray = gray
            self._prev_timestamp = timestamp
            return None

        flow = cv2.calcOpticalFlowFarneback(
            self._prev_gray, gray, None, 0.5, 3, self._lk_params["winSize"], 3, 5, 0
        )

        dx = np.median(flow[..., 0])
        dy = np.median(flow[..., 1])

        magnitude = np.sqrt(dx**2 + dy**2)
        confidence = min(1.0, magnitude / 10.0)

        flow_data = FlowData(
            dx=dx,
            dy=dy,
            confidence=confidence,
            timestamp=timestamp,
            altitude=self._altitude,
        )

        self._prev_gray = gray
        self._prev_timestamp = timestamp

        return flow_data

    def get_position_estimate(self) -> PositionEstimate:
        dt = time.time() - self._prev_timestamp

        if dt > 0 and self._altitude > 0.1:
            pixels_per_meter = 1000 / self._altitude

            dx_m = self._prev_gray if hasattr(self, "_flow_dx") else 0
            dy_m = self._prev_gray if hasattr(self, "_flow_dy") else 0

            if hasattr(self, "_flow_dx"):
                self._velocity[0] = self._flow_dx / pixels_per_meter / dt
                self._velocity[1] = self._flow_dy / pixels_per_meter / dt
                self._position[0] += self._velocity[0] * dt
                self._position[1] += self._velocity[1] * dt

        quality = int(self._quality_threshold * 100)

        return PositionEstimate(
            x=self._position[0],
            y=self._position[1],
            z=self._altitude,
            vx=self._velocity[0],
            vy=self._velocity[1],
            vz=0.0,
            quality=quality,
            timestamp=time.time(),
        )

    def reset_position(self) -> None:
        self._position = np.array([0.0, 0.0, 0.0])
        self._velocity = np.array([0.0, 0.0, 0.0])


class PMW3901Sensor:
    def __init__(self, spi_bus: int = 0, spi_device: int = 1) -> None:
        self._spi_bus = spi_bus
        self._spi_device = spi_device
        self._spi = None
        self._enabled = False

        self._position = np.array([0.0, 0.0])
        self._velocity = np.array([0.0, 0.0])
        self._last_update = time.time()

    def open(self) -> bool:
        try:
            import spidev

            self._spi = spidev.SpiDev()
            self._spi.open(self._spi_bus, self._spi_device)
            self._spi.max_speed_hz = 2000000
            self._spi.mode = 3
            self._enabled = True
            logger.info("PMW3901 sensor opened")
            return True
        except Exception as e:
            logger.error(f"PMW3901 open failed: {e}")
            return False

    def close(self) -> None:
        self._enabled = False
        if self._spi:
            self._spi.close()

    def read_motion(self) -> Optional[Tuple[int, int]]:
        if not self._enabled or not self._spi:
            return None

        try:
            self._spi.xfer2([0x01])
            time.sleep(0.001)
            data = self._spi.xfer2([0x00] * 6)

            if data[0] & 0x80:
                delta_x = data[1] if not (data[0] & 0x10) else -data[1]
                delta_y = data[2] if not (data[0] & 0x20) else -data[2]
                return delta_x, delta_y
        except Exception as e:
            logger.debug(f"PMW3901 read error: {e}")

        return None

    def update(self) -> Optional[FlowData]:
        motion = self.read_motion()
        if not motion:
            return None

        dx, dy = motion
        dt = time.time() - self._last_update
        self._last_update = time.time()

        self._velocity[0] = dx / dt if dt > 0 else 0
        self._velocity[1] = dy / dt if dt > 0 else 0
        self._position += self._velocity * dt

        return FlowData(
            dx=float(dx), dy=float(dy), confidence=1.0, timestamp=time.time()
        )
