from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Dict, Optional

from .models import (
    ControlCommand,
    ParkingState,
    SafetyDecision,
    SlotEstimate,
    SlotSide,
)


@dataclass
class ParkingFSMConfig:
    approach_speed: int = 22
    reverse_arc_speed: int = -18
    align_speed: int = -14
    final_reverse_speed: int = -12
    exit_speed: int = 20

    # 실차에서 반드시 부호 확인
    left_slot_steer: int = 45
    right_slot_steer: int = -45
    align_max_steer: int = 12
    exit_turn_steer: int = 0

    staging_pass_distance: float = 0.20
    target_depth: float = 0.90
    target_rear_stop_distance: float = 0.14

    stop_settle_time: float = 0.30
    steer_settle_time: float = 0.60
    counter_trigger_deg: float = 35.0
    align_yaw_tolerance_deg: float = 8.0
    final_yaw_tolerance_deg: float = 5.0
    final_lateral_tolerance: float = 0.08
    final_depth_tolerance: float = 0.08

    yaw_gain_deg_per_rad: float = 20.0
    lateral_gain_deg_per_m: float = 12.0
    reverse_steer_sign: float = -1.0

    hold_time: float = 4.0
    exit_straight_time: float = 1.0
    out_line_extra_time: float = 0.4
    maximum_adjustments: int = 2

    state_timeouts: Dict[ParkingState, float] = field(
        default_factory=lambda: {
            ParkingState.SEARCH: 30.0,
            ParkingState.APPROACH: 15.0,
            ParkingState.STAGING: 3.0,
            ParkingState.STEER_IN: 2.0,
            ParkingState.REVERSE_ARC: 8.0,
            ParkingState.COUNTER_STEER: 2.0,
            ParkingState.ALIGN: 8.0,
            ParkingState.FINAL_REVERSE: 8.0,
            ParkingState.VERIFY: 3.0,
            ParkingState.HOLD: 6.0,
            ParkingState.EXIT_STRAIGHT: 5.0,
            ParkingState.OUT_CONFIRM: 15.0,
        }
    )


class ParkingFSM:
    """Sensor-driven stop-steer-drive parking state machine."""

    def __init__(self, config: ParkingFSMConfig):
        self.config = config
        self.state = ParkingState.IDLE
        self.state_started_at = time.monotonic()
        self.locked_side = SlotSide.UNKNOWN
        self.adjustment_count = 0
        self.out_line_seen_at: Optional[float] = None
        self.emergency_reason = ""

    def start(self, now: Optional[float] = None) -> None:
        if self.state in (
            ParkingState.IDLE,
            ParkingState.DONE,
            ParkingState.EMERGENCY_STOP,
        ):
            self.locked_side = SlotSide.UNKNOWN
            self.adjustment_count = 0
            self.out_line_seen_at = None
            self.emergency_reason = ""
            self._transition(
                ParkingState.SEARCH,
                time.monotonic() if now is None else now,
            )

    def reset(self, now: Optional[float] = None) -> None:
        self.locked_side = SlotSide.UNKNOWN
        self.adjustment_count = 0
        self.out_line_seen_at = None
        self.emergency_reason = ""
        self._transition(
            ParkingState.IDLE,
            time.monotonic() if now is None else now,
        )

    def emergency_stop(self, reason: str, now: float) -> None:
        self.emergency_reason = reason
        self._transition(ParkingState.EMERGENCY_STOP, now)

    def step(
        self,
        now: float,
        slot: Optional[SlotEstimate],
        safety: SafetyDecision,
        out_line_detected: bool,
        required_sensors_fresh: bool,
        rear_minimum: float,
    ) -> ControlCommand:
        if self.state == ParkingState.IDLE:
            return ControlCommand(0, 0, "idle")
        if self.state == ParkingState.DONE:
            return ControlCommand(0, 0, "done")
        if self.state == ParkingState.EMERGENCY_STOP:
            return ControlCommand(
                0, 0, f"emergency:{self.emergency_reason}"
            )

        if not required_sensors_fresh:
            self.emergency_stop("required_sensor_timeout", now)
            return ControlCommand(0, 0, "sensor_timeout")
        if safety.hard_stop:
            self.emergency_stop(safety.reason, now)
            return ControlCommand(0, 0, safety.reason)

        timeout = self.config.state_timeouts.get(self.state)
        if timeout is not None and now - self.state_started_at > timeout:
            self.emergency_stop(
                f"{self.state.value.lower()}_timeout", now
            )
            return ControlCommand(0, 0, "state_timeout")

        if self.state == ParkingState.SEARCH:
            if slot is not None:
                self.locked_side = slot.side
                self._transition(ParkingState.APPROACH, now)
            return self._scaled(
                0,
                self.config.approach_speed,
                safety,
                "search_slot",
            )

        if self.state == ParkingState.APPROACH:
            if slot is None:
                return ControlCommand(0, 0, "slot_temporarily_lost")

            self.locked_side = slot.side
            if slot.entrance_x <= -self.config.staging_pass_distance:
                self._transition(ParkingState.STAGING, now)
                return ControlCommand(0, 0, "staging_reached")

            return self._scaled(
                0,
                self.config.approach_speed,
                safety,
                "approach_and_pass_slot",
            )

        if self.state == ParkingState.STAGING:
            if slot is None:
                return ControlCommand(0, 0, "wait_for_lidar_reconfirm")
            if now - self.state_started_at >= self.config.stop_settle_time:
                self._transition(ParkingState.STEER_IN, now)
            return ControlCommand(0, 0, "stop_before_steering")

        if self.state == ParkingState.STEER_IN:
            steer_in = self._steer_into_slot()
            if now - self.state_started_at >= self.config.steer_settle_time:
                self._transition(ParkingState.REVERSE_ARC, now)
            return ControlCommand(
                steer_in, 0, "set_maximum_entry_steer"
            )

        if self.state == ParkingState.REVERSE_ARC:
            if slot is None:
                return ControlCommand(
                    self._steer_into_slot(),
                    0,
                    "slot_lost_during_reverse_arc",
                )

            if abs(slot.axis_error()) <= math.radians(
                self.config.counter_trigger_deg
            ):
                self._transition(ParkingState.COUNTER_STEER, now)
                return ControlCommand(0, 0, "counter_steer_trigger")

            return self._scaled(
                self._steer_into_slot(),
                self.config.reverse_arc_speed,
                safety,
                "reverse_arc",
            )

        if self.state == ParkingState.COUNTER_STEER:
            steer = -self._steer_into_slot()
            if now - self.state_started_at >= self.config.steer_settle_time:
                self._transition(ParkingState.ALIGN, now)
            return ControlCommand(steer, 0, "set_counter_steer")

        if self.state == ParkingState.ALIGN:
            if slot is None:
                return ControlCommand(0, 0, "slot_lost_during_align")

            yaw_error = slot.axis_error()
            lateral_error = slot.lateral_error(
                self.config.target_depth
            )
            steer = self._alignment_steer(
                yaw_error,
                lateral_error,
                safety.steering_bias,
            )

            if (
                abs(yaw_error)
                <= math.radians(
                    self.config.align_yaw_tolerance_deg
                )
                and abs(lateral_error)
                <= self.config.final_lateral_tolerance * 1.5
            ):
                self._transition(ParkingState.FINAL_REVERSE, now)
                return ControlCommand(0, 0, "alignment_reached")

            return self._scaled(
                steer,
                self.config.align_speed,
                safety,
                "align_in_slot",
            )

        if self.state == ParkingState.FINAL_REVERSE:
            if slot is None:
                if (
                    math.isfinite(rear_minimum)
                    and rear_minimum
                    <= self.config.target_rear_stop_distance
                ):
                    self._transition(ParkingState.VERIFY, now)
                    return ControlCommand(
                        0, 0, "rear_target_reached"
                    )
                return ControlCommand(
                    0, 0, "slot_lost_final_reverse"
                )

            yaw_error = slot.axis_error()
            lateral_error = slot.lateral_error(
                self.config.target_depth
            )
            depth_error = slot.depth_error(
                self.config.target_depth
            )

            if (
                depth_error <= self.config.final_depth_tolerance
                or (
                    math.isfinite(rear_minimum)
                    and rear_minimum
                    <= self.config.target_rear_stop_distance
                )
            ):
                self._transition(ParkingState.VERIFY, now)
                return ControlCommand(
                    0, 0, "target_depth_reached"
                )

            steer = self._alignment_steer(
                yaw_error,
                lateral_error,
                safety.steering_bias,
            )
            return self._scaled(
                steer,
                self.config.final_reverse_speed,
                safety,
                "final_reverse",
            )

        if self.state == ParkingState.VERIFY:
            if now - self.state_started_at < self.config.stop_settle_time:
                return ControlCommand(
                    0, 0, "settle_before_verify"
                )

            if slot is None:
                if (
                    math.isfinite(rear_minimum)
                    and rear_minimum
                    <= self.config.target_rear_stop_distance
                ):
                    self._transition(ParkingState.HOLD, now)
                    return ControlCommand(
                        0, 0, "verify_by_rear_ultrasonic"
                    )
                self.emergency_stop(
                    "cannot_verify_parked_pose", now
                )
                return ControlCommand(0, 0, "verify_failed")

            valid = (
                abs(slot.axis_error())
                <= math.radians(
                    self.config.final_yaw_tolerance_deg
                )
                and abs(
                    slot.lateral_error(self.config.target_depth)
                )
                <= self.config.final_lateral_tolerance
            )
            if valid:
                self._transition(ParkingState.HOLD, now)
                return ControlCommand(0, 0, "park_verified")

            if self.adjustment_count < self.config.maximum_adjustments:
                self.adjustment_count += 1
                self._transition(ParkingState.ALIGN, now)
                return ControlCommand(
                    0, 0, "one_more_alignment"
                )

            self.emergency_stop(
                "park_verification_failed", now
            )
            return ControlCommand(0, 0, "verify_failed")

        if self.state == ParkingState.HOLD:
            if now - self.state_started_at >= self.config.hold_time:
                self._transition(ParkingState.EXIT_STRAIGHT, now)
            return ControlCommand(0, 0, "hold_four_seconds")

        if self.state == ParkingState.EXIT_STRAIGHT:
            if (
                now - self.state_started_at
                >= self.config.exit_straight_time
            ):
                self._transition(ParkingState.OUT_CONFIRM, now)
            return self._scaled(
                0,
                self.config.exit_speed,
                safety,
                "exit_straight",
            )

        if self.state == ParkingState.OUT_CONFIRM:
            if out_line_detected and self.out_line_seen_at is None:
                self.out_line_seen_at = now

            if (
                self.out_line_seen_at is not None
                and now - self.out_line_seen_at
                >= self.config.out_line_extra_time
            ):
                self._transition(ParkingState.DONE, now)
                return ControlCommand(0, 0, "out_line_cleared")

            return self._scaled(
                self.config.exit_turn_steer,
                self.config.exit_speed,
                safety,
                "search_out_line",
            )

        self.emergency_stop("unhandled_state", now)
        return ControlCommand(0, 0, "unhandled_state")

    def _steer_into_slot(self) -> int:
        if self.locked_side == SlotSide.LEFT:
            return self.config.left_slot_steer
        if self.locked_side == SlotSide.RIGHT:
            return self.config.right_slot_steer
        return 0

    def _alignment_steer(
        self,
        yaw_error: float,
        lateral_error: float,
        safety_bias: float,
    ) -> int:
        raw = self.config.reverse_steer_sign * (
            self.config.yaw_gain_deg_per_rad * yaw_error
            + self.config.lateral_gain_deg_per_m * lateral_error
        )
        raw += safety_bias
        raw = max(
            -self.config.align_max_steer,
            min(self.config.align_max_steer, raw),
        )
        return int(round(raw))

    @staticmethod
    def _scaled(
        steer: int,
        speed: int,
        safety: SafetyDecision,
        reason: str,
    ) -> ControlCommand:
        return ControlCommand(
            int(steer),
            int(round(speed * safety.speed_scale)),
            reason,
        )

    def _transition(
        self,
        new_state: ParkingState,
        now: float,
    ) -> None:
        self.state = new_state
        self.state_started_at = now
