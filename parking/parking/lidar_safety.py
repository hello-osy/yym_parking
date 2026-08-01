"""Narrow LiDAR self-return masking for safety-sector queries only."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np


Sector = tuple[float, float, float, float]
Point2D = tuple[float, float]


@dataclass(frozen=True)
class SelfMaskRegion:
    """Measured, vehicle-owned reflection region in the active point frame."""

    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    reason: str = ""

    def __post_init__(self) -> None:
        bounds = (self.x_min, self.x_max, self.y_min, self.y_max)
        if not self.name or not all(math.isfinite(value) for value in bounds):
            raise ValueError("LiDAR self-mask must have a name and finite bounds")
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError("LiDAR self-mask bounds must be increasing")

    def contains(self, points: np.ndarray) -> np.ndarray:
        return (
            (points[:, 0] >= self.x_min)
            & (points[:, 0] <= self.x_max)
            & (points[:, 1] >= self.y_min)
            & (points[:, 1] <= self.y_max)
        )


@dataclass(frozen=True)
class SectorMinimum:
    distance: float
    point: Optional[Point2D]


@dataclass(frozen=True)
class MaskedSectorMinimum:
    before_mask: SectorMinimum
    after_mask: SectorMinimum
    filtered_count: int
    matched_regions: tuple[str, ...]


class LidarSafetySelfMask:
    """Mask only a copy of points selected for one safety-sector minimum."""

    def __init__(self, enabled: bool, regions: Sequence[SelfMaskRegion]) -> None:
        self.enabled = bool(enabled)
        self.regions = tuple(regions)
        names = tuple(region.name for region in self.regions)
        if len(set(names)) != len(names):
            raise ValueError("LiDAR self-mask region names must be unique")

    def minimum_in_sector(
        self, points: np.ndarray, sector: Sector
    ) -> MaskedSectorMinimum:
        """Return before/after minima without mutating ``points``.

        Callers must retain the original point cloud for slot and line detection.
        """
        sector_points = self._points_in_sector(points, sector)
        before = self._minimum(sector_points)
        if not self.enabled or sector_points.size == 0:
            return MaskedSectorMinimum(before, before, 0, ())

        excluded = np.zeros(sector_points.shape[0], dtype=bool)
        matched_regions: list[str] = []
        for region in self.regions:
            in_region = region.contains(sector_points)
            if in_region.any():
                matched_regions.append(region.name)
                excluded |= in_region

        return MaskedSectorMinimum(
            before_mask=before,
            after_mask=self._minimum(sector_points[~excluded]),
            filtered_count=int(excluded.sum()),
            matched_regions=tuple(matched_regions),
        )

    @staticmethod
    def _points_in_sector(points: np.ndarray, sector: Sector) -> np.ndarray:
        array = np.asarray(points, dtype=np.float64)
        if array.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        if array.ndim != 2 or array.shape[1] != 2:
            raise ValueError("LiDAR points must have shape (N, 2)")
        x_min, x_max, y_min, y_max = sector
        selected = (
            (array[:, 0] >= x_min)
            & (array[:, 0] <= x_max)
            & (array[:, 1] >= y_min)
            & (array[:, 1] <= y_max)
        )
        return array[selected]

    @staticmethod
    def _minimum(points: np.ndarray) -> SectorMinimum:
        if points.size == 0:
            return SectorMinimum(math.inf, None)
        squared_ranges = (points**2).sum(axis=1)
        index = int(np.argmin(squared_ranges))
        point = points[index]
        return SectorMinimum(
            math.sqrt(float(squared_ranges[index])),
            (float(point[0]), float(point[1])),
        )
