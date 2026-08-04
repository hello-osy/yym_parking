from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import statistics
import time
from typing import Dict, Optional

from .models import ParkingState, SafetyDecision, SlotSide


@dataclass
class UltrasonicSafetyConfig:
    median_window: int = 5
    valid_minimum: float = 0.02
    valid_maximum: float = 4.0
    stale_timeout: float = 0.6
    slow_distance: float = 0.25
    hard_stop_distance: float = 0.12
    enable_alignment_steering_bias: bool = False
    side_bias_gain_deg_per_m: float = 20.0
    side_bias_limit_deg: float = 8.0
    reverse_bias_sign: float = -1.0


class UltrasonicSafety:
    """
    Mapping:
      1 front-left, 2 front-right, 3 side-left,
      4 side-right, 5 rear-left, 6 rear-right
    """

    def __init__(self, config: UltrasonicSafetyConfig):
        self.config = config
        self._values: Dict[int, deque[float]] = {
            i: deque(maxlen=config.median_window)
            for i in range(1, 7)
        }
        self._stamps = {i: -math.inf for i in range(1, 7)}

    def update(
        self,
        sensor_index: int,
        distance: float,
        stamp: Optional[float] = None,
    ) -> None:
        if sensor_index not in self._values:
            raise ValueError("sensor_index must be in [1, 6]")
        if (
            math.isfinite(distance)
            and self.config.valid_minimum
            <= distance
            <= self.config.valid_maximum
        ):
            self._values[sensor_index].append(float(distance))
            self._stamps[sensor_index] = (
                time.monotonic() if stamp is None else stamp
            )

    def filtered(self, sensor_index: int) -> float:
        values = self._values[sensor_index]
        return float(statistics.median(values)) if values else math.inf

    def all_filtered(self) -> list[float]:
        return [self.filtered(i) for i in range(1, 7)]

    def all_fresh(self, now: float) -> bool:
        return all(
            now - self._stamps[i] <= self.config.stale_timeout
            for i in range(1, 7)
        )

    def rear_minimum(self) -> float:
        return min(self.filtered(5), self.filtered(6))

    def assess(
        self,
        requested_speed: int,
        state: ParkingState,
        slot_side: SlotSide,
    ) -> SafetyDecision:
        active = self._active_sensors(
            requested_speed, state, slot_side
        )
        distances = {i: self.filtered(i) for i in active}
        minimum = min(distances.values(), default=math.inf)

        if minimum < self.config.hard_stop_distance:
            sensor = min(distances, key=distances.get, default=-1)
            return SafetyDecision(
                True,
                0.0,
                0.0,
                f"ultrasonic_{sensor}_hard_stop",
                minimum,
            )

        if minimum < self.config.slow_distance:
            span = max(
                self.config.slow_distance
                - self.config.hard_stop_distance,
                1e-6,
            )
            scale = (
                minimum - self.config.hard_stop_distance
            ) / span
            scale = max(0.15, min(1.0, scale))
        else:
            scale = 1.0

        bias = 0.0
        if (
            self.config.enable_alignment_steering_bias
            and state in (
                ParkingState.ALIGN,
                ParkingState.FINAL_REVERSE,
            )
        ):
            left = self.filtered(3)
            right = self.filtered(4)
            if math.isfinite(left) and math.isfinite(right):
                bias = (
                    self.config.reverse_bias_sign
                    * self.config.side_bias_gain_deg_per_m
                    * (right - left)
                )
                bias = max(
                    -self.config.side_bias_limit_deg,
                    min(self.config.side_bias_limit_deg, bias),
                )

        return SafetyDecision(
            False,
            scale,
            bias,
            "slow_zone" if scale < 1.0 else "clear",
            minimum,
        )

    @staticmethod
    def _active_sensors(
        requested_speed: int,
        state: ParkingState,
        slot_side: SlotSide,
    ) -> tuple[int, ...]:
        if requested_speed > 0:
            return (1, 2, 3, 4)
        if requested_speed < 0:
            sensors = [3, 4, 5, 6]
            if state == ParkingState.REVERSE_ARC:
                # During rear entry, the front swings outward.
                if slot_side == SlotSide.LEFT:
                    sensors.append(2)
                elif slot_side == SlotSide.RIGHT:
                    sensors.append(1)
            return tuple(sensors)
        return (1, 2, 3, 4, 5, 6)
