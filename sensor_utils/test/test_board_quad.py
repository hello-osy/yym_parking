"""Hardware-free checks of the checkerboard sheet detector and drift maths.

The riskiest part of the two-camera procedure is that the two steps ask the
operator to move different things -- the sheet in step 1, the camera in step 2 --
so the same measured drift has to produce *opposite* instructions.  Getting that
backwards would have the operator chase the error instead of cancelling it, so
it is pinned down here.

Run with:  python3 -m pytest sensor_utils/test/test_board_quad.py -v
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sensor_utils.board_quad import (  # noqa: E402
    board_hints,
    camera_hints,
    detect,
    drift,
    order_corners,
)

WIDTH, HEIGHT = 640, 360
FLOOR, SHEET, SQUARE = 150, 225, 45


def synthetic_frame(centre=(320.0, 190.0), size=110.0, rotation_deg=0.0,
                    squares=7, seed=0, clutter=True):
    """A speckled floor with a checkerboard sheet lying on it."""
    rng = np.random.default_rng(seed)
    frame = np.clip(rng.normal(FLOOR, 11.0, (HEIGHT, WIDTH)), 90, 195).astype(np.uint8)

    if clutter:
        # A bright box in the corner, like the cardboard in the real scene: big
        # and bright but with nothing checkerboard-like inside it.
        frame[10:90, 470:630] = np.clip(
            rng.normal(205, 6.0, (80, 160)), 150, 245).astype(np.uint8)

    side = int(round(size))
    margin = max(3, int(round(size * 0.08)))
    sheet = np.full((side, side), SHEET, np.uint8)
    step = (side - 2 * margin) / squares
    for row in range(squares):
        for column in range(squares):
            if (row + column) % 2:
                continue
            y = int(round(margin + row * step))
            x = int(round(margin + column * step))
            sheet[y:y + int(step), x:x + int(step)] = SQUARE

    angle = np.deg2rad(rotation_deg)
    rotation = np.array([[np.cos(angle), -np.sin(angle)],
                         [np.sin(angle), np.cos(angle)]])
    anchor = np.array([side / 2.0, side / 2.0])
    offset = np.array(centre, dtype=float) - rotation @ anchor
    transform = np.hstack([rotation, offset.reshape(2, 1)])

    cv2.warpAffine(sheet, transform, (WIDTH, HEIGHT), dst=frame,
                   flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
    return frame


def corners_of(frame):
    found = detect(frame)
    assert found is not None, 'sheet was not detected'
    return found['corners']


# -------------------------------------------------------------------- detection

def test_sheet_is_found_among_clutter():
    found = detect(synthetic_frame())
    assert found is not None
    assert abs(found['centre'][0] - 320.0) < 4.0
    assert abs(found['centre'][1] - 190.0) < 4.0
    assert found['squares'] >= 8


def test_bright_clutter_alone_is_not_mistaken_for_the_sheet():
    """The cardboard box is bright and rectangular but holds no squares."""
    rng = np.random.default_rng(0)
    frame = np.clip(rng.normal(FLOOR, 11.0, (HEIGHT, WIDTH)), 90, 195).astype(np.uint8)
    frame[10:90, 470:630] = np.clip(
        rng.normal(205, 6.0, (80, 160)), 150, 245).astype(np.uint8)
    assert detect(frame) is None


def test_empty_floor_finds_nothing():
    rng = np.random.default_rng(3)
    floor = np.clip(rng.normal(FLOOR, 11.0, (HEIGHT, WIDTH)), 90, 195).astype(np.uint8)
    assert detect(floor) is None


@pytest.mark.parametrize('size', [80.0, 110.0, 150.0])
def test_sheet_is_found_over_a_range_of_apparent_sizes(size):
    """The two cameras see the same sheet at very different scales."""
    found = detect(synthetic_frame(size=size))
    assert found is not None
    assert abs(found['centre'][0] - 320.0) < 5.0


def test_corner_order_is_stable_under_rotation():
    """Per-corner comparison is meaningless if the ordering shuffles."""
    first = corners_of(synthetic_frame(rotation_deg=0.0))
    second = corners_of(synthetic_frame(rotation_deg=6.0))
    # Corner 0 stays the top-left-most in both, so the pairs stay matched and
    # no single corner jumps across the sheet.
    assert np.linalg.norm(first[0] - second[0]) < 25.0
    assert np.argmin(first.sum(axis=1)) == 0
    assert np.argmin(second.sum(axis=1)) == 0


def test_order_corners_is_idempotent():
    quad = np.array([[10.0, 10.0], [90.0, 12.0], [92.0, 88.0], [8.0, 86.0]])
    once = order_corners(quad)
    assert np.allclose(once, order_corners(once))
    assert np.allclose(once, order_corners(np.roll(quad, 2, axis=0)))


# ------------------------------------------------------------------------ drift

def test_identical_views_report_no_drift():
    corners = corners_of(synthetic_frame())
    measured = drift(corners, corners)
    assert measured['max_px'] < 1e-9
    assert abs(measured['rotation']) < 1e-9
    assert abs(measured['scale'] - 1.0) < 1e-9


@pytest.mark.parametrize('shift', [-12.0, 12.0])
def test_drift_tracks_a_sideways_move(shift):
    reference = corners_of(synthetic_frame())
    live = corners_of(synthetic_frame(centre=(320.0 + shift, 190.0)))
    measured = drift(reference, live)
    assert abs(measured['dx'] - shift) < 2.5
    assert abs(measured['dy']) < 2.5


@pytest.mark.parametrize('angle', [-5.0, 5.0])
def test_drift_tracks_a_rotation(angle):
    reference = corners_of(synthetic_frame())
    live = corners_of(synthetic_frame(rotation_deg=angle))
    measured = drift(reference, live)
    assert abs(measured['rotation'] - angle) < 1.5


def test_drift_tracks_scale():
    reference = corners_of(synthetic_frame(size=110.0))
    live = corners_of(synthetic_frame(size=132.0))
    measured = drift(reference, live)
    assert measured['scale'] > 1.1


# --------------------------------------------------- the two steps disagree

@pytest.mark.parametrize('shift,board,camera', [
    (12.0, 'LEFT', 'RIGHT'),
    (-12.0, 'RIGHT', 'LEFT'),
])
def test_the_two_steps_give_opposite_sideways_instructions(shift, board, camera):
    """Step 1 moves the sheet, step 2 moves the camera; they must disagree.

    A sheet seen too far right is pushed left in step 1, but in step 2 the sheet
    is fixed and it is the camera that has to swing right to catch up.
    """
    reference = corners_of(synthetic_frame())
    live = corners_of(synthetic_frame(centre=(320.0 + shift, 190.0)))
    measured = drift(reference, live)

    assert board_hints(measured, tolerance_px=3.0) == [board]
    assert camera_hints(measured, tolerance_px=3.0) == [camera]


@pytest.mark.parametrize('shift,board,camera', [
    (12.0, 'AWAY', 'DOWN'),
    (-12.0, 'CLOSER', 'UP'),
])
def test_the_two_steps_give_opposite_depth_instructions(shift, board, camera):
    reference = corners_of(synthetic_frame())
    live = corners_of(synthetic_frame(centre=(320.0, 190.0 + shift)))
    measured = drift(reference, live)

    assert board_hints(measured, tolerance_px=3.0) == [board]
    assert camera_hints(measured, tolerance_px=3.0) == [camera]


def test_no_instructions_when_already_aligned():
    corners = corners_of(synthetic_frame())
    measured = drift(corners, corners)
    assert board_hints(measured, tolerance_px=3.0) == []
    assert camera_hints(measured, tolerance_px=3.0) == []


def test_small_drift_inside_tolerance_is_not_reported():
    reference = corners_of(synthetic_frame())
    live = corners_of(synthetic_frame(centre=(321.0, 190.0)))
    measured = drift(reference, live)
    assert board_hints(measured, tolerance_px=6.0) == []
