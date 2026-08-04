from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
from typing import Tuple
from .geometry import axis_alignment_error


class SlotSide(IntEnum):
    UNKNOWN = 0
    LEFT = 1
    RIGHT = -1


class ParkingState(str, Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    STAGING = "STAGING"
    STEER_IN = "STEER_IN"
    REVERSE_ARC = "REVERSE_ARC"
    COUNTER_STEER = "COUNTER_STEER"
    ALIGN = "ALIGN"
    FINAL_REVERSE = "FINAL_REVERSE"
    VERIFY = "VERIFY"
    HOLD = "HOLD"
    EXIT_STRAIGHT = "EXIT_STRAIGHT"
    OUT_CONFIRM = "OUT_CONFIRM"
    DONE = "DONE"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass(frozen=True)
class SlotEstimate:
    side: SlotSide
    entrance_x: float
    entrance_y: float
    inward_yaw: float
    width: float
    confidence: float
    stamp: float

    @property
    def inward_unit(self) -> Tuple[float, float]:
        return math.cos(self.inward_yaw), math.sin(self.inward_yaw)

    @property
    def normal_unit(self) -> Tuple[float, float]:
        ix, iy = self.inward_unit
        return -iy, ix

    def target_point(self, target_depth: float) -> Tuple[float, float]:
        ix, iy = self.inward_unit
        return (
            self.entrance_x + target_depth * ix,
            self.entrance_y + target_depth * iy,
        )

    def axis_error(self) -> float:
        return axis_alignment_error(self.inward_yaw)

    def lateral_error(self, target_depth: float) -> float:
        tx, ty = self.target_point(target_depth)
        nx, ny = self.normal_unit
        return tx * nx + ty * ny

    def depth_error(self, target_depth: float) -> float:
        tx, ty = self.target_point(target_depth)
        ix, iy = self.inward_unit
        return tx * ix + ty * iy


@dataclass(frozen=True)
class CameraHint:
    side: SlotSide
    confidence: float
    stamp: float


@dataclass(frozen=True)
class SafetyDecision:
    hard_stop: bool
    speed_scale: float
    steering_bias: float
    reason: str
    minimum_active_distance: float


@dataclass(frozen=True)
class ControlCommand:
    steer: int
    speed: int
    reason: str
