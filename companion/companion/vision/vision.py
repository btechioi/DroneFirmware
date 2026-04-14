"""Computer vision for obstacle detection and landing pad recognition."""

from __future__ import annotations

import logging
import time
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass

logger = logging.getLogger(__name__)

try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    cv2 = None


@dataclass
class DetectionResult:
    x: float
    y: float
    z: float
    class_id: int
    confidence: float
    size: Tuple[int, int]


@dataclass
class Obstacle:
    x: float
    y: float
    z: float
    width: float
    height: float
    depth: float
    confidence: float


@dataclass
class LandingPad:
    x: float
    y: float
    z: float
    yaw: float
    size: float
    confidence: float


class ObjectDetector:
    def __init__(self) -> None:
        self._net = None
        self._classes: List[str] = []
        self._conf_threshold = 0.5
        self._nms_threshold = 0.4
        self._enabled = False

        self._model_path = ""
        self._config_path = ""

    def load_model(
        self, model_path: str, config_path: str = "", labels_path: str = ""
    ) -> bool:
        if not CV2_AVAILABLE:
            logger.error("OpenCV not available")
            return False

        try:
            if config_path and model_path.endswith(".weights"):
                self._net = cv2.dnn.readNet(model_path, config_path)
            elif model_path.endswith(".weights"):
                logger.error("Config file required for .weights models")
                return False
            else:
                self._net = cv2.dnn.readNet(model_path)

            self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            if labels_path:
                with open(labels_path, "r") as f:
                    self._classes = [line.strip() for line in f.readlines()]

            self._enabled = True
            logger.info(f"Object detector loaded: {model_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to load model: {e}")
            return False

    def set_thresholds(self, conf: float = 0.5, nms: float = 0.4) -> None:
        self._conf_threshold = conf
        self._nms_threshold = nms

    def detect(self, frame) -> List[DetectionResult]:
        if not self._enabled or self._net is None:
            return []

        try:
            blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True)
            self._net.setInput(blob)

            outputs = self._net.forward(self._net.getUnconnectedOutLayersNames())

            results = []
            height, width = frame.shape[:2]

            for output in outputs:
                for detection in output:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]

                    if confidence > self._conf_threshold:
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        results.append(
                            DetectionResult(
                                x=float(center_x - width / 2),
                                y=float(center_y - height / 2),
                                z=0.0,
                                class_id=class_id,
                                confidence=float(confidence),
                                size=(w, h),
                            )
                        )

            return results
        except Exception as e:
            logger.error(f"Detection error: {e}")
            return []

    def is_enabled(self) -> bool:
        return self._enabled


class ObstacleDetector:
    def __init__(self) -> None:
        self._prev_frame = None
        self._enabled = False
        self._threshold = 25
        self._min_area = 500

    def enable(self) -> None:
        self._enabled = True
        logger.info("Obstacle detector enabled")

    def disable(self) -> None:
        self._enabled = False

    def detect_obstacles(self, frame) -> List[Obstacle]:
        if not self._enabled or self._prev_frame is None:
            self._prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        diff = cv2.absdiff(self._prev_frame, gray)
        thresh = cv2.threshold(diff, self._threshold, 255, cv2.THRESH_BINARY)[1]

        dilated = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(
            dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        obstacles = []
        height, width = frame.shape[:2]

        for contour in contours:
            if cv2.contourArea(contour) < self._min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            obstacles.append(
                Obstacle(
                    x=float(x + w / 2 - width / 2),
                    y=float(y + h / 2 - height / 2),
                    z=0.0,
                    width=float(w),
                    height=float(h),
                    depth=0.0,
                    confidence=0.8,
                )
            )

        self._prev_frame = gray
        return obstacles

    def set_sensitivity(self, threshold: int, min_area: int) -> None:
        self._threshold = max(5, min(100, threshold))
        self._min_area = max(100, min_area)


class LandingPadDetector:
    LANDING_PAD_COLORS = {
        "orange": ((5, 100, 100), (20, 255, 255)),
        "yellow": ((20, 100, 100), (30, 255, 255)),
        "red": ((0, 100, 100), (10, 255, 255)),
        "blue": ((100, 100, 100), (130, 255, 255)),
    }

    def __init__(self) -> None:
        self._enabled = False
        self._color_range = self.LANDING_PAD_COLORS["orange"]
        self._expected_size = (100, 100)

    def enable(self) -> None:
        self._enabled = True
        logger.info("Landing pad detector enabled")

    def disable(self) -> None:
        self._enabled = False

    def set_color(self, color: str) -> None:
        if color in self.LANDING_PAD_COLORS:
            self._color_range = self.LANDING_PAD_COLORS[color]
            logger.info(f"Landing pad color set to {color}")

    def detect(self, frame) -> Optional[LandingPad]:
        if not self._enabled:
            return None

        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self._color_range[0], self._color_range[1])

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 500:
                    continue

                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h

                    if 0.8 < aspect_ratio < 1.2:
                        center_x = x + w // 2
                        center_y = y + h // 2

                        height, width = frame.shape[:2]

                        return LandingPad(
                            x=float(center_x - width / 2),
                            y=float(center_y - height / 2),
                            z=0.0,
                            yaw=0.0,
                            size=float(w),
                            confidence=min(1.0, area / 10000),
                        )
        except Exception as e:
            logger.error(f"Landing pad detection error: {e}")

        return None


class DepthEstimator:
    def __init__(self) -> None:
        self._enabled = False
        self._baseline = 0.06
        self._focal_length = 700

    def enable(self) -> None:
        self._enabled = True
        logger.info("Depth estimator enabled")

    def disable(self) -> None:
        self._enabled = False

    def estimate_depth(self, disparity: float) -> float:
        if not self._enabled or disparity < 1.0:
            return 0.0

        depth = (self._baseline * self._focal_length) / disparity
        return max(0.1, min(10.0, depth))
