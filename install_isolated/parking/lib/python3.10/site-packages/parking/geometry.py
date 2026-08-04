from __future__ import annotations

import math
from typing import Iterable, Tuple
import numpy as np


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def axis_alignment_error(axis_yaw: float) -> float:
    a = wrap_pi(axis_yaw)
    b = wrap_pi(axis_yaw + math.pi)
    return a if abs(a) <= abs(b) else b


def transform_points_2d(
    points: np.ndarray,
    translation_x: float,
    translation_y: float,
    yaw: float,
) -> np.ndarray:
    if points.size == 0:
        return points.reshape((-1, 2))
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]], dtype=np.float64)
    translation = np.array([translation_x, translation_y], dtype=np.float64)
    return points @ rotation.T + translation


def fit_line_pca(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] != 2:
        raise ValueError("At least two 2D points are required.")
    centroid = points.mean(axis=0)
    centered = points - centroid
    covariance = centered.T @ centered / max(points.shape[0] - 1, 1)
    values, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, int(np.argmax(values))]
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        raise ValueError("Degenerate point set.")
    return centroid, direction / norm


def contiguous_true_runs(mask: Iterable[bool]) -> list[tuple[int, int]]:
    values = [bool(v) for v in mask]
    runs: list[tuple[int, int]] = []
    start = None
    for index, value in enumerate(values):
        if value and start is None:
            start = index
        elif not value and start is not None:
            runs.append((start, index - 1))
            start = None
    if start is not None:
        runs.append((start, len(values) - 1))
    return runs
