from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional
import numpy as np

from .geometry import contiguous_true_runs, fit_line_pca, wrap_pi
from .models import SlotEstimate, SlotSide


@dataclass
class LidarDetectorConfig:
    x_min: float = -2.5
    x_max: float = 3.0
    lateral_min: float = 0.25
    lateral_max: float = 2.0
    max_processing_range: float = 4.0
    bin_size: float = 0.04
    minimum_points_per_bin: int = 2
    maximum_closed_gap_bins: int = 2
    minimum_vehicle_segment_length: float = 0.25
    expected_slot_width: float = 0.95
    minimum_slot_width: float = 0.65
    maximum_slot_width: float = 1.25
    width_sigma: float = 0.18
    boundary_band: float = 0.12
    minimum_boundary_points: int = 5
    expected_axis_tolerance_deg: float = 35.0
    minimum_confidence: float = 0.35


class LidarSlotDetector:
    """Occupied-gap-occupied detector with PCA boundary fitting.

    LaserScan bearings use rear=0, right=+pi/2, front=+/-pi, left=-pi/2.
    Detector points use the vehicle frame: x forward and y left.
    """

    def __init__(self, config: LidarDetectorConfig):
        self.config = config

    @staticmethod
    def scan_to_points(
        ranges: Iterable[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> np.ndarray:
        points = []
        for index, distance in enumerate(ranges):
            if not math.isfinite(distance):
                continue
            if distance < range_min or distance > range_max:
                continue
            angle = angle_min + index * angle_increment
            points.append(
                (-distance * math.cos(angle), -distance * math.sin(angle))
            )
        return (
            np.asarray(points, dtype=np.float64)
            if points
            else np.empty((0, 2), dtype=np.float64)
        )

    def detect(self, points_base: np.ndarray, stamp: float) -> Optional[SlotEstimate]:
        if points_base.ndim != 2 or points_base.shape[1] != 2:
            raise ValueError("points_base must be an Nx2 array")
        if points_base.shape[0] < 10:
            return None

        distance = np.linalg.norm(points_base, axis=1)
        common = (
            (distance <= self.config.max_processing_range)
            & (points_base[:, 0] >= self.config.x_min)
            & (points_base[:, 0] <= self.config.x_max)
        )
        filtered = points_base[common]

        candidates = []
        for side in (SlotSide.LEFT, SlotSide.RIGHT):
            candidate = self._detect_side(filtered, side, stamp)
            if candidate is not None:
                candidates.append(candidate)

        if not candidates:
            return None
        best = max(candidates, key=lambda item: item.confidence)
        return best if best.confidence >= self.config.minimum_confidence else None

    def _detect_side(
        self,
        points: np.ndarray,
        side: SlotSide,
        stamp: float,
    ) -> Optional[SlotEstimate]:
        signed_lateral = points[:, 1] * int(side)
        mask = (
            (signed_lateral >= self.config.lateral_min)
            & (signed_lateral <= self.config.lateral_max)
        )
        side_points = points[mask]
        if side_points.shape[0] < 10:
            return None

        bins = np.arange(
            self.config.x_min,
            self.config.x_max + self.config.bin_size,
            self.config.bin_size,
        )
        indices = np.digitize(side_points[:, 0], bins) - 1
        valid = (indices >= 0) & (indices < bins.size - 1)
        indices = indices[valid]
        side_points = side_points[valid]

        counts = np.bincount(indices, minlength=bins.size - 1)
        occupied = counts >= self.config.minimum_points_per_bin
        occupied = self._close_small_gaps(
            occupied, self.config.maximum_closed_gap_bins
        )

        minimum_bins = max(
            1,
            int(math.ceil(
                self.config.minimum_vehicle_segment_length
                / self.config.bin_size
            )),
        )
        runs = [
            run for run in contiguous_true_runs(occupied)
            if run[1] - run[0] + 1 >= minimum_bins
        ]

        best = None
        for first, second in zip(runs, runs[1:]):
            gap_start = bins[first[1] + 1]
            gap_end = bins[second[0]]
            width = gap_end - gap_start
            if not (
                self.config.minimum_slot_width
                <= width
                <= self.config.maximum_slot_width
            ):
                continue
            estimate = self._build_estimate(
                side_points, side, gap_start, gap_end, width, stamp
            )
            if estimate is not None and (
                best is None or estimate.confidence > best.confidence
            ):
                best = estimate
        return best

    def _build_estimate(
        self,
        side_points: np.ndarray,
        side: SlotSide,
        gap_start: float,
        gap_end: float,
        width: float,
        stamp: float,
    ) -> Optional[SlotEstimate]:
        boundaries = [
            side_points[
                np.abs(side_points[:, 0] - x) <= self.config.boundary_band
            ]
            for x in (gap_start, gap_end)
        ]

        line_angles = []
        support = 0
        for boundary in boundaries:
            support += int(boundary.shape[0])
            if boundary.shape[0] < self.config.minimum_boundary_points:
                continue
            try:
                _, direction = fit_line_pca(boundary)
            except ValueError:
                continue
            line_angles.append(math.atan2(direction[1], direction[0]))

        expected_axis = int(side) * math.pi / 2.0
        if line_angles:
            sin2 = sum(math.sin(2.0 * a) for a in line_angles)
            cos2 = sum(math.cos(2.0 * a) for a in line_angles)
            axis = 0.5 * math.atan2(sin2, cos2)
            if math.sin(axis) * int(side) < 0.0:
                axis = wrap_pi(axis + math.pi)
        else:
            axis = expected_axis

        entrance_x = 0.5 * (gap_start + gap_end)
        entrance_y = int(side) * float(
            np.percentile(np.abs(side_points[:, 1]), 20.0)
        )

        width_error = (
            width - self.config.expected_slot_width
        ) / max(self.config.width_sigma, 1e-6)
        width_score = math.exp(-0.5 * width_error * width_error)

        expected_error = abs(wrap_pi(axis - expected_axis))
        tolerance = math.radians(self.config.expected_axis_tolerance_deg)
        axis_score = math.exp(
            -0.5 * (expected_error / max(tolerance, 1e-6)) ** 2
        )
        support_score = min(
            1.0,
            support / max(2 * self.config.minimum_boundary_points, 1),
        )
        confidence = (
            0.50 * width_score
            + 0.25 * axis_score
            + 0.25 * support_score
        )

        return SlotEstimate(
            side=side,
            entrance_x=entrance_x,
            entrance_y=entrance_y,
            inward_yaw=axis,
            width=width,
            confidence=float(confidence),
            stamp=stamp,
        )

    @staticmethod
    def _close_small_gaps(mask: np.ndarray, maximum_gap: int) -> np.ndarray:
        result = mask.astype(bool).copy()
        index = 0
        while index < result.size:
            if result[index]:
                index += 1
                continue
            start = index
            while index < result.size and not result[index]:
                index += 1
            end = index - 1
            length = end - start + 1
            has_left = start > 0 and result[start - 1]
            has_right = index < result.size and result[index]
            if has_left and has_right and length <= maximum_gap:
                result[start:end + 1] = True
        return result
