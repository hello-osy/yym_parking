"""Find the printed checkerboard sheet in a frame and compare it against a reference.

Used by the two-camera alignment procedure.  The low camera barely moves, so it
defines where the sheet belongs; the high camera is the one on the tripod that
drifts.  Putting the sheet back where the low camera says it lived, and then
aiming the high camera until it sees the sheet as it used to, transfers the low
camera's stability onto the high camera.

The sheet is found as a bright convex quadrilateral whose interior is full of
evenly sized dark squares.  That fingerprint survives both viewpoints: measured
on the same sheet, the high camera sees it head-on as 107x107 px and the low
camera sees it edge-on as 159x73 px, yet both report 27 dark squares covering
0.28 of the sheet.  Chessboard corner detection was tried first and is not
usable here -- at this apparent size ``findChessboardCorners`` fails outright on
the low camera, and ``findChessboardCornersSB`` returns a different partial grid
for almost every pattern size it is asked for.

Only the sheet's four corners are used, so nothing depends on knowing how many
squares are printed on it.
"""

import cv2
import numpy as np

# An unbroken checkerboard covers a fixed fraction of its sheet, and its squares
# are all the same size.  Those two facts reject the lookalikes: a big patch of
# speckled floor matches the shape but covers only 0.06, and a scuff near the
# skirting board has squares of wildly mixed sizes.
MIN_DARK_SQUARES = 8
MAX_SIZE_SPREAD = 0.45
COVERAGE_RANGE = (0.12, 0.45)
BRIGHT_PERCENTILES = (80, 85, 90, 93, 96)


def order_corners(quad):
    """Put four corners in a stable order: top-left, top-right, bottom-right, bottom-left.

    The comparison is per-corner, so the ordering has to come out the same on
    every frame or the drift is nonsense.
    """
    points = np.asarray(quad, dtype=float).reshape(4, 2)
    centre = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - centre[1], points[:, 0] - centre[0])
    clockwise = points[np.argsort(angles)]
    # Rotate so the corner nearest the image origin comes first.
    start = int(np.argmin(clockwise.sum(axis=1)))
    return np.roll(clockwise, -start, axis=0)


def _quad_candidates(gray):
    """Bright convex quadrilaterals, over a sweep of brightness thresholds.

    A single threshold is not enough: the sheet's contrast against the floor
    changes with the viewing angle, and the low camera sees it much dimmer.
    """
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    kernel = np.ones((7, 7), np.uint8)
    max_area = gray.size * 0.35
    seen = []
    for threshold in np.percentile(blurred, BRIGHT_PERCENTILES):
        mask = cv2.morphologyEx(
            (blurred > threshold).astype(np.uint8), cv2.MORPH_CLOSE, kernel
        )
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if not 1500 <= area <= max_area:
                continue
            approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                seen.append((approx.reshape(4, 2).astype(float), float(area)))
    return seen


def _checker_score(gray, quad, area):
    """Measure how checkerboard-like the inside of a quadrilateral is."""
    mask = np.zeros(gray.shape, np.uint8)
    cv2.fillConvexPoly(mask, quad.astype(np.int32), 1)
    inside = gray[mask == 1]
    if inside.size < 500:
        return None

    threshold, _ = cv2.threshold(
        inside.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    dark = ((gray <= threshold) & (mask == 1)).astype(np.uint8)
    # 4-connectivity: checkerboard squares touch only at their corners, and
    # 8-connectivity would weld them into one blob.
    count, _, stats, _ = cv2.connectedComponentsWithStats(dark, connectivity=4)
    areas = np.array(
        [stats[i, cv2.CC_STAT_AREA] for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 12],
        dtype=float,
    )
    if len(areas) < MIN_DARK_SQUARES:
        return None

    spread = float(areas.std() / areas.mean())
    coverage = float(areas.sum() / area)
    if spread > MAX_SIZE_SPREAD or not COVERAGE_RANGE[0] <= coverage <= COVERAGE_RANGE[1]:
        return None
    return {'squares': int(len(areas)), 'size_spread': spread, 'coverage': coverage}


def detect(gray):
    """Locate the checkerboard sheet, or return None.

    Returns the ordered corners plus a few descriptive numbers.  When several
    candidates survive, the densest one wins: a lookalike large enough to hold
    the same number of dark blobs is always far sparser than the real sheet.
    """
    best = None
    for quad, area in _quad_candidates(gray):
        score = _checker_score(gray, quad, area)
        if score is None:
            continue
        density = score['squares'] / area
        if best is None or density > best['density']:
            corners = order_corners(quad)
            best = {
                'corners': corners,
                'centre': corners.mean(axis=0),
                'area': area,
                'density': density,
                **score,
            }
    return best


def drift(reference_corners, live_corners):
    """How the sheet moved in the image between reference and now.

    ``dx``/``dy`` are in pixels at the sheet's centre, ``rotation`` in degrees,
    ``scale`` as a ratio.  Scale is worth watching on the low camera: it says
    the sheet moved towards or away from the camera, which no amount of sliding
    it sideways will fix.
    """
    reference = np.asarray(reference_corners, dtype=np.float32).reshape(4, 2)
    live = np.asarray(live_corners, dtype=np.float32).reshape(4, 2)
    displacement = live - reference
    distances = np.linalg.norm(displacement, axis=1)

    result = {
        'max_px': float(distances.max()),
        'mean_px': float(distances.mean()),
        'dx': float(displacement[:, 0].mean()),
        'dy': float(displacement[:, 1].mean()),
        'rotation': 0.0,
        'scale': 1.0,
    }

    transform, _ = cv2.estimateAffinePartial2D(reference, live, method=cv2.LMEDS)
    if transform is not None:
        centre = reference.mean(axis=0)
        moved = transform[:, :2] @ centre + transform[:, 2]
        result['dx'], result['dy'] = (moved - centre).tolist()
        result['rotation'] = float(np.degrees(np.arctan2(transform[1, 0], transform[0, 0])))
        result['scale'] = float(np.hypot(transform[0, 0], transform[1, 0]))
    return result


def board_hints(measured, tolerance_px):
    """Which way to slide the sheet, for the low-camera step.

    The sheet is what moves here, so it has to travel *against* its own error:
    seen too far right, it must go left.
    """
    hints = []
    if abs(measured['dx']) > tolerance_px / 2:
        hints.append('LEFT' if measured['dx'] > 0 else 'RIGHT')
    if abs(measured['dy']) > tolerance_px / 2:
        hints.append('AWAY' if measured['dy'] > 0 else 'CLOSER')
    if abs(measured['rotation']) > 0.5:
        hints.append('TURN CCW' if measured['rotation'] > 0 else 'TURN CW')
    return hints


def camera_hints(measured, tolerance_px):
    """Which way to turn the camera, for the high-camera step.

    Opposite sense to :func:`board_hints`, and this is the distinction worth
    being careful about: here the sheet stays put and the camera moves, so the
    image having drifted right means the camera drifted left and must come back
    right.
    """
    hints = []
    if abs(measured['dx']) > tolerance_px / 2:
        hints.append('RIGHT' if measured['dx'] > 0 else 'LEFT')
    if abs(measured['dy']) > tolerance_px / 2:
        hints.append('DOWN' if measured['dy'] > 0 else 'UP')
    if abs(measured['rotation']) > 0.5:
        hints.append('CW' if measured['rotation'] > 0 else 'CCW')
    return hints
