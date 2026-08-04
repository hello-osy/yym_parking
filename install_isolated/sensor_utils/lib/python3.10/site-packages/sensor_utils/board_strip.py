"""Measure the checkerboard strip along the bottom of the high camera's view.

The camera looks down at the road and a checkerboard sheet sits on the floor
just inside the bottom of the frame, one row of squares deep.  Three numbers
taken from that strip are enough to tell whether the tripod still aims where it
did:

    centre_offset_px   the strip's centre minus the image centre -- the
                       continuous form of "four squares each side of centre"
    top_edge_y         how far down the frame the sheet's top edge falls, which
                       is what "about one row visible" means
    roll_deg           the tilt of that top edge

All three are compared against a saved reference rather than against absolute
targets, so whatever the aim looks like when the reference is taken becomes the
definition of correct.

Both thresholds are derived from the frame itself rather than fixed, so the
measurement does not move when the lighting does: dimming a frame to 55% of its
brightness recovers the same strip to within a pixel.
"""

import cv2
import numpy as np

# Directions are named for what you do to the camera.  The image moves opposite
# to the camera, so the sign of each delta is already the way to turn.
AXES = (
    ('yaw', 'centre_offset_px', 'RIGHT', 'LEFT'),
    ('pitch', 'top_edge_y', 'DOWN', 'UP'),
    ('roll', 'roll_deg', 'CW', 'CCW'),
)
DEFAULT_TOLERANCES = {
    'centre_offset_px': 4.0,
    'top_edge_y': 4.0,
    'roll_deg': 0.5,
}


def _otsu(values):
    threshold, _ = cv2.threshold(
        values.reshape(-1, 1).astype(np.uint8), 0, 255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU,
    )
    return threshold


def _fit_top_edge(mask, x0, x1, y_offset):
    """Least-squares line through the top edge of the mask, rejecting outliers.

    Returns ``(slope, intercept)`` in full-image coordinates, or None.
    """
    xs, ys = [], []
    for column in range(x0, x1):
        rows = np.where(mask[:, column])[0]
        if len(rows):
            xs.append(float(column))
            ys.append(float(y_offset + rows.min()))
    if len(xs) < 20:
        return None

    xs = np.array(xs)
    ys = np.array(ys)
    design = np.vstack([xs, np.ones_like(xs)]).T
    slope, intercept = np.linalg.lstsq(design, ys, rcond=None)[0]

    # One pass of outlier rejection: a scuff on the floor or a shadow at the
    # sheet's corner otherwise drags the whole line and shows up as fake roll.
    residual = ys - (slope * xs + intercept)
    spread = residual.std()
    if spread > 1e-6:
        keep = np.abs(residual) < 2.5 * spread
        if keep.sum() >= 20:
            slope, intercept = np.linalg.lstsq(design[keep], ys[keep], rcond=None)[0]
    return float(slope), float(intercept)


def _square_size(dark_mask, x0, x1):
    """Width of one square, from the spacing of the black squares in the row.

    Black squares sit two square-widths apart, so half the median gap between
    their centres is the square size.

    Two kinds of junk have to be dropped first, both of which otherwise halve
    the answer: at the edge of the sheet a square merges with the row beneath it
    into a double-width blob, and the row beneath pokes into the bottom of the
    frame as a sliver a few pixels tall.  Keeping only blobs of typical width
    and height removes both.
    """
    # 4-connectivity, not 8: squares on a checkerboard touch only at their
    # corners, so 8-connectivity welds a whole two-row patch into one blob and
    # the square size comes out as the width of the entire patch.
    count, _, stats, centroids = cv2.connectedComponentsWithStats(dark_mask, connectivity=4)
    blobs = [
        (float(centroids[i][0]), float(centroids[i][1]),
         float(stats[i, cv2.CC_STAT_WIDTH]),
         float(stats[i, cv2.CC_STAT_HEIGHT]))
        for i in range(1, count)
        if stats[i, cv2.CC_STAT_AREA] >= 25 and x0 <= centroids[i][0] <= x1
    ]
    if len(blobs) < 3:
        return None, [b[0] for b in blobs]

    median_width = float(np.median([b[2] for b in blobs]))
    median_height = float(np.median([b[3] for b in blobs]))
    # Keep the topmost row only.  A second row peeking in at the bottom of the
    # frame is offset by one square, so mixing the two rows together halves
    # every gap and reports squares at half their true size.
    top_row = min(b[1] for b in blobs)
    centres = sorted(
        centre_x for centre_x, centre_y, width, height in blobs
        if 0.6 * median_width <= width <= 1.6 * median_width
        and height >= 0.5 * median_height
        and centre_y - top_row < 0.75 * median_height
    )
    if len(centres) < 3:
        return None, centres

    return float(np.median(np.diff(centres)) / 2.0), centres


def _square_split(centres, square_px, x0, x1, image_centre):
    """How many squares fall either side of the image centre line.

    Purely for display: it is the four-a-side rule the check was described in,
    while the verdict itself uses the continuous centre offset.
    """
    if square_px is None or not centres:
        return None

    # Square boundaries sit half a square either side of a black square centre;
    # extend that grid across the sheet.
    anchor = centres[len(centres) // 2] - square_px / 2.0
    first = anchor - square_px * np.ceil((anchor - x0) / square_px)
    boundaries = np.arange(first, x1 + square_px * 0.5, square_px)
    boundaries = boundaries[(boundaries >= x0 - 1) & (boundaries <= x1 + 1)]
    if len(boundaries) < 2:
        return None

    mids = (boundaries[:-1] + boundaries[1:]) / 2.0
    return {
        'left': int((mids < image_centre).sum()),
        'right': int((mids > image_centre).sum()),
        'boundaries': [float(b) for b in boundaries],
    }


def measure(gray, roi_fraction=0.28, min_paper_area=300, min_contrast=25.0):
    """Measure the checkerboard strip, or return None if it is not there.

    ``gray`` is a single-channel image.  ``roi_fraction`` is how much of the
    frame, measured up from the bottom, to search.  ``min_contrast`` is how far
    above the floor's brightness the sheet has to sit to count as found.
    """
    height, width = gray.shape[:2]
    y_offset = int(height * (1.0 - roi_fraction))
    roi = gray[y_offset:]
    if roi.size == 0:
        return None

    # Split the sheet off the floor halfway between the floor's level and the
    # brightest thing present.  Otsu is wrong for this step: the floor outnumbers
    # the sheet roughly twenty to one, and maximising inter-class variance then
    # prefers to cut the floor distribution in half rather than isolate the
    # small bright cluster, which swallows half the floor into the "sheet".
    floor_level = float(np.median(roi))
    brightest = float(np.percentile(roi, 99.5))
    paper_threshold = (floor_level + brightest) / 2.0
    dark_threshold = _otsu(roi)

    # The sheet is white paper *and* black squares, and both differ from the
    # floor between them.  Taking the union keeps the sheet a single solid
    # region; thresholding only the white would let the squares cut it into
    # disconnected pieces whenever the white margin is thin.
    sheet_mask = ((roi > paper_threshold) | (roi <= dark_threshold)).astype(np.uint8)
    # Speckle on the floor crosses one threshold or the other here and there;
    # the sheet is solid, so opening removes the former and leaves the latter.
    sheet_mask = cv2.morphologyEx(
        sheet_mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(sheet_mask, connectivity=8)
    if count < 2:
        return None

    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x0 = int(stats[index, cv2.CC_STAT_LEFT])
    box_top = int(stats[index, cv2.CC_STAT_TOP])
    box_width = int(stats[index, cv2.CC_STAT_WIDTH])
    box_height = int(stats[index, cv2.CC_STAT_HEIGHT])
    x1 = x0 + box_width
    if stats[index, cv2.CC_STAT_AREA] < min_paper_area or box_width < 20:
        return None

    mask = labels == index
    # On a frame with no sheet at all, the brightest patch of floor still forms
    # a blob.  Requiring the white part of the winner to stand clear of the
    # floor's own level separates "the sheet is here" from "a bright scuff".
    white = roi[mask & (roi > paper_threshold)]
    if white.size < 50 or float(white.mean()) - floor_level < min_contrast:
        return None

    fit = _fit_top_edge(mask, x0, x1, y_offset)
    if fit is None:
        return None
    slope, intercept = fit

    # Threshold the squares inside the sheet's own bounding box.  Otsu over the
    # whole search region is dominated by floor, and the threshold it picks
    # there is high enough to swallow the gaps between squares so that adjacent
    # ones merge into one blob and the measured square size comes out short.
    sheet = roi[box_top:box_top + box_height, x0:x1]
    dark = np.zeros_like(roi, dtype=np.uint8)
    # `<=`, not `<`: cv2's Otsu classifies with `> thresh`, and on a clean
    # two-level region it returns the dark level itself, so `<` selects nothing.
    dark[box_top:box_top + box_height, x0:x1] = sheet <= _otsu(sheet)

    image_centre = width / 2.0
    square_px, centres = _square_size(dark, x0, x1)

    return {
        'centre_offset_px': float((x0 + x1) / 2.0 - image_centre),
        'top_edge_y': float(slope * image_centre + intercept),
        'roll_deg': float(np.degrees(np.arctan(slope))),
        'square_px': square_px,
        'strip_x0': float(x0),
        'strip_x1': float(x1),
        'top_edge_slope': slope,
        'top_edge_intercept': intercept,
        'split': _square_split(centres, square_px, x0, x1, image_centre),
    }


def compare(reference, live, tolerances=None):
    """Difference between a live measurement and the saved reference.

    Every delta is signed so that its sign is the direction to turn the camera:
    the image moves opposite to the camera, so a strip that drifted right means
    the camera drifted left and has to come back right.
    """
    tolerances = {**DEFAULT_TOLERANCES, **(tolerances or {})}
    axes = {}
    hints = []
    within = True

    for name, key, positive, negative in AXES:
        delta = float(live[key] - reference[key])
        tolerance = float(tolerances[key])
        ok = abs(delta) <= tolerance
        within &= ok
        if not ok:
            hints.append(positive if delta > 0 else negative)
        axes[name] = {
            'delta': delta,
            'tolerance': tolerance,
            'ok': ok,
            'direction': positive if delta > 0 else negative,
        }

    # The pitch drift is easier to picture in rows of squares than in pixels,
    # which is how the check was described in the first place.
    square_px = live.get('square_px') or reference.get('square_px')
    if square_px:
        axes['pitch']['rows'] = axes['pitch']['delta'] / square_px

    return {
        'axes': axes,
        'hints': hints,
        'verdict': 'PASS' if within else 'ADJUST',
    }
