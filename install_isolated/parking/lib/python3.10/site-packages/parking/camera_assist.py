from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Optional
import numpy as np

from .models import CameraHint, SlotSide

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


@dataclass
class CameraAssistConfig:
    enable_experimental_slot_hint: bool = False
    lower_roi_ratio: float = 0.45
    canny_low: int = 60
    canny_high: int = 160
    hough_threshold: int = 45
    minimum_line_length_ratio: float = 0.20
    maximum_line_gap: int = 25
    out_line_minimum_length_ratio: float = 0.45
    out_line_maximum_angle_deg: float = 12.0
    out_line_minimum_y_ratio: float = 0.55
    out_line_required_frames: int = 3


class CameraAssist:
    """
    Classical camera helper.

    Slot hint is disabled by default because FOV must be validated first.
    OUT-line detection remains available for the exit stage.
    """

    def __init__(self, config: CameraAssistConfig):
        self.config = config
        self._out_line_hits = 0

    def detect_slot_hint(self, bgr: np.ndarray) -> Optional[CameraHint]:
        if not self.config.enable_experimental_slot_hint:
            return None
        if cv2 is None or bgr is None or bgr.size == 0:
            return None

        height, width = bgr.shape[:2]
        roi = bgr[int(height * self.config.lower_roi_ratio):, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(
            gray, self.config.canny_low, self.config.canny_high
        )

        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            self.config.hough_threshold,
            minLineLength=max(
                10, int(width * self.config.minimum_line_length_ratio)
            ),
            maxLineGap=self.config.maximum_line_gap,
        )
        if lines is None:
            return None

        centers = []
        for x1, y1, x2, y2 in lines[:, 0]:
            angle = abs(math.degrees(math.atan2(y2 - y1, x2 - x1)))
            angle = min(angle, 180.0 - angle)
            if 35.0 <= angle <= 85.0:
                centers.append(0.5 * (x1 + x2))

        if len(centers) < 2:
            return None

        center = float(np.median(centers))
        side = SlotSide.LEFT if center < width * 0.5 else SlotSide.RIGHT
        confidence = min(1.0, len(centers) / 8.0)
        return CameraHint(
            side=side,
            confidence=confidence,
            stamp=time.monotonic(),
        )

    def detect_out_line(self, bgr: np.ndarray) -> tuple[bool, float]:
        if cv2 is None or bgr is None or bgr.size == 0:
            self._out_line_hits = 0
            return False, 0.0

        height, width = bgr.shape[:2]
        roi = bgr[int(height * self.config.out_line_minimum_y_ratio):, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(
            gray, self.config.canny_low, self.config.canny_high
        )

        minimum_length = int(
            width * self.config.out_line_minimum_length_ratio
        )
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            self.config.hough_threshold,
            minLineLength=max(10, minimum_length),
            maxLineGap=self.config.maximum_line_gap,
        )

        best_score = 0.0
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                dx = float(x2 - x1)
                dy = float(y2 - y1)
                length = math.hypot(dx, dy)
                angle = abs(math.degrees(math.atan2(dy, dx)))
                angle = min(angle, 180.0 - angle)
                if angle <= self.config.out_line_maximum_angle_deg:
                    best_score = max(
                        best_score,
                        min(1.0, length / max(width, 1)),
                    )

        if best_score >= self.config.out_line_minimum_length_ratio:
            self._out_line_hits += 1
        else:
            self._out_line_hits = 0

        return (
            self._out_line_hits
            >= self.config.out_line_required_frames,
            best_score,
        )
