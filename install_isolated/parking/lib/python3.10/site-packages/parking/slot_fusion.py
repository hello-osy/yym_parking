from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import statistics
from typing import Optional

from .geometry import wrap_pi
from .models import CameraHint, SlotEstimate


@dataclass
class SlotFusionConfig:
    history_size: int = 8
    required_count: int = 5
    maximum_x_std: float = 0.10
    maximum_y_std: float = 0.10
    maximum_width_std: float = 0.10
    maximum_yaw_deviation_deg: float = 12.0
    minimum_confidence: float = 0.45
    camera_hint_timeout: float = 0.5
    camera_confidence_bonus: float = 0.10
    camera_side_mismatch_penalty: float = 0.25


class SlotFusion:
    """
    Stabilize LiDAR estimates and optionally use a camera hint.

    Camera is deliberately assistive, not mandatory, because the front camera
    may not see a side parking slot at every starting pose.
    """

    def __init__(self, config: SlotFusionConfig):
        self.config = config
        self._lidar_history: deque[SlotEstimate] = deque(
            maxlen=config.history_size
        )
        self._camera_hint: Optional[CameraHint] = None

    def reset(self) -> None:
        self._lidar_history.clear()
        self._camera_hint = None

    def update_lidar(self, estimate: Optional[SlotEstimate]) -> None:
        if estimate is not None:
            self._lidar_history.append(estimate)

    def update_camera(self, hint: Optional[CameraHint]) -> None:
        if hint is not None:
            self._camera_hint = hint

    def stable_estimate(self, now: float) -> Optional[SlotEstimate]:
        if len(self._lidar_history) < self.config.required_count:
            return None

        latest_side = self._lidar_history[-1].side
        samples = [
            item for item in self._lidar_history
            if item.side == latest_side
        ]
        if len(samples) < self.config.required_count:
            return None

        xs = [item.entrance_x for item in samples]
        ys = [item.entrance_y for item in samples]
        widths = [item.width for item in samples]

        mean_sin = statistics.fmean(
            math.sin(item.inward_yaw) for item in samples
        )
        mean_cos = statistics.fmean(
            math.cos(item.inward_yaw) for item in samples
        )
        yaw = math.atan2(mean_sin, mean_cos)
        yaw_deviations = [
            abs(wrap_pi(item.inward_yaw - yaw))
            for item in samples
        ]

        if statistics.pstdev(xs) > self.config.maximum_x_std:
            return None
        if statistics.pstdev(ys) > self.config.maximum_y_std:
            return None
        if statistics.pstdev(widths) > self.config.maximum_width_std:
            return None
        if max(yaw_deviations) > math.radians(
            self.config.maximum_yaw_deviation_deg
        ):
            return None

        confidence = statistics.fmean(
            item.confidence for item in samples
        )
        if (
            self._camera_hint is not None
            and now - self._camera_hint.stamp
            <= self.config.camera_hint_timeout
        ):
            if self._camera_hint.side == latest_side:
                confidence += (
                    self.config.camera_confidence_bonus
                    * self._camera_hint.confidence
                )
            else:
                confidence -= self.config.camera_side_mismatch_penalty

        confidence = max(0.0, min(1.0, confidence))
        if confidence < self.config.minimum_confidence:
            return None

        return SlotEstimate(
            side=latest_side,
            entrance_x=statistics.median(xs),
            entrance_y=statistics.median(ys),
            inward_yaw=yaw,
            width=statistics.median(widths),
            confidence=confidence,
            stamp=samples[-1].stamp,
        )
