#!/usr/bin/env python3
"""Bring the high camera back to its reference aim, using the low camera as the ruler.

The low camera is bolted down and does not move; the high camera sits on a
tripod and drifts.  That asymmetry is what makes this work:

    step 1  put the checkerboard sheet down and slide it until the LOW camera
            sees it exactly where it saw it when the reference was taken.  The
            sheet is now physically back in its old place, to within a pixel,
            with no tape marks on the floor.
    step 2  leave the sheet alone and turn the HIGH camera until it sees the
            sheet the way it used to.

Step 1 is what the old single-camera check could not do: it had to assume the
sheet had been put back correctly, and any error in that assumption came out as
a fake camera drift.

    python3 sensor_utils/scripts/align_camera.py --save   # once, when all is well
    python3 sensor_utils/scripts/align_camera.py          # every time after

No ROS and no camera calibration.  Stop camera_node first -- this opens both
camera devices directly.

Exit codes (with --headless): 0 aligned, 1 needs adjusting, 2 could not measure.
"""

import argparse
import sys
import time
from datetime import datetime
from glob import glob
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'sensor_utils'))

from sensor_utils.board_quad import (  # noqa: E402
    board_hints,
    camera_hints,
    detect,
    drift,
)

DEFAULT_REFERENCE = '~/.camera_aim/two_camera.yaml'
GREEN = (0, 200, 0)
RED = (0, 0, 235)
YELLOW = (0, 220, 255)
CYAN = (255, 220, 0)
WHITE = (255, 255, 255)
GREY = (130, 130, 130)

STEPS = (
    # key in the reference file, which camera to watch, what the operator moves
    ('low', 'BOARD', 'slide the SHEET'),
    ('high', 'CAMERA', 'turn the CAMERA'),
)


# ----------------------------------------------------------------- the cameras

def device_name(path):
    try:
        video_name = Path(path).resolve().name
        return (Path('/sys/class/video4linux') / video_name / 'name').read_text().strip()
    except OSError:
        return ''


def capture_nodes(name_filter='C920'):
    """Capture nodes for the named camera, in stable USB order.

    /dev/videoN numbering shuffles between boots and each C920 also exposes a
    metadata node; the by-path links ending in video-index0 are exactly the
    capture nodes and keep USB port order, so "high" stays the same camera.
    """
    return [
        link for link in sorted(glob('/dev/v4l/by-path/*video-index0'))
        if not name_filter or name_filter.lower() in device_name(link).lower()
    ]


def open_cameras(width, height):
    """Open the high and low cameras, or return (None, message)."""
    nodes = capture_nodes()
    if len(nodes) < 2:
        return None, (f'Need two C920s, found {len(nodes)}. '
                      'Check both are plugged in and enumerated.')

    captures = {}
    for side, path in zip(('high', 'low'), nodes):
        capture = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not capture.isOpened():
            for opened in captures.values():
                opened.release()
            return None, (f'{Path(path).resolve()} ({side}) is busy. Stop camera_node or '
                          'any ros2 launch using the cameras, then retry.')
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        captures[side] = capture
    return captures, None


def read_frames(captures):
    frames = {}
    for side, capture in captures.items():
        ok, frame = capture.read()
        frames[side] = frame if ok else None
    return frames


# ------------------------------------------------------------- the measurement

def median_corners(history):
    """Median corner positions over the buffered frames.

    Detection is steady to about a twentieth of a pixel, but the odd frame drops
    the sheet entirely; the median keeps one bad frame from moving the verdict.
    """
    if not history:
        return None
    return np.median(np.stack(history), axis=0)


def measure_side(frame):
    if frame is None:
        return None
    return detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))


def divisions_from_squares(squares):
    """Squares per side, from how many dark ones were counted.

    A checkerboard of n by n squares shows about half of them dark, so
    n = sqrt(2 * dark).  Only used to draw the ghost, so being one out is
    cosmetic rather than a measurement error.
    """
    if not squares:
        return 8
    return int(max(2, min(12, round((2.0 * squares) ** 0.5))))


# ---------------------------------------------------------------- the reference

def save_reference(path, corners_by_side, squares_by_side=None):
    record = {'saved_at': datetime.now().isoformat(timespec='seconds')}
    squares_by_side = squares_by_side or {}
    for side, corners in corners_by_side.items():
        record[side] = [[float(x), float(y)] for x, y in corners]
        if squares_by_side.get(side):
            record[f'{side}_squares'] = int(squares_by_side[side])
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(record, sort_keys=False))
    return record


def load_reference(path):
    path = Path(path).expanduser()
    if not path.exists():
        raise FileNotFoundError(path)
    record = yaml.safe_load(path.read_text()) or {}
    reference = {}
    for side in ('high', 'low'):
        corners = record.get(side)
        if not corners or len(corners) != 4:
            raise ValueError(f'{path} has no usable {side} corners')
        reference[side] = np.array(corners, dtype=float)
        reference[f'{side}_divisions'] = divisions_from_squares(record.get(f'{side}_squares'))
    reference['saved_at'] = record.get('saved_at', 'unknown time')
    return reference


# ------------------------------------------------------------------ presenting

def evaluate(reference, corners, side, tolerance):
    """Drift plus verdict for one camera, or None if the sheet is not visible."""
    if corners is None:
        return None
    measured = drift(reference[side], corners)
    measured['verdict'] = 'OK' if measured['max_px'] <= tolerance else 'ADJUST'
    measured['hints'] = (board_hints if side == 'low' else camera_hints)(measured, tolerance)
    return measured


def summarise(step_index, results):
    side, _, action = STEPS[step_index]
    measured = results.get(side)
    if measured is None:
        return f'step {step_index + 1} ({side}): sheet not visible'
    line = (f'step {step_index + 1} ({side}, {action}): {measured["verdict"]}  '
            f'off {measured["max_px"]:.1f}px  '
            f'dx {measured["dx"]:+.1f}  dy {measured["dy"]:+.1f}  '
            f'rot {measured["rotation"]:+.2f}deg  scale {measured["scale"]:.3f}')
    if measured['hints']:
        line += '  ->  ' + ' + '.join(measured['hints'])
    return line


def dim(panel, x, y, width, height, strength=0.6):
    x2, y2 = min(panel.shape[1], x + width), min(panel.shape[0], y + height)
    if x < x2 and y < y2:
        panel[y:y2, x:x2] = (panel[y:y2, x:x2] * (1.0 - strength)).astype(panel.dtype)


def draw_ghost(panel, quad, divisions, alpha=0.45):
    """Paint the reference position as a translucent checkerboard to aim at.

    An outline alone turned out to be hard to line a sheet up against -- it
    shows where the target is but not how the sheet should sit inside it.  A
    checkerboard ghost gives the eye something to match square-for-square, so
    rotation and skew are as obvious as position.
    """
    divisions = max(2, int(divisions))
    cell = 16
    template = np.zeros((divisions * cell, divisions * cell), np.uint8)
    for row in range(divisions):
        for column in range(divisions):
            if (row + column) % 2 == 0:
                template[row * cell:(row + 1) * cell,
                         column * cell:(column + 1) * cell] = 255

    source = np.array([[0, 0], [template.shape[1], 0],
                       [template.shape[1], template.shape[0]], [0, template.shape[0]]],
                      dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, quad.astype(np.float32))
    height, width = panel.shape[:2]
    warped = cv2.warpPerspective(template, homography, (width, height))
    inside = cv2.warpPerspective(np.full_like(template, 255), homography, (width, height))

    tint = np.zeros_like(panel)
    tint[warped > 127] = CYAN
    tint[(inside > 127) & (warped <= 127)] = (255, 255, 255)
    mask = (inside > 127)[:, :, None]
    blended = (panel * (1.0 - alpha) + tint * alpha).astype(panel.dtype)
    np.copyto(panel, blended, where=mask)
    cv2.polylines(panel, [quad.astype(np.int32)], True, CYAN, 2, cv2.LINE_AA)


def draw_inset(panel, frame, target, live, divisions, colour, scale=3):
    """Magnified corner view of the sheet and its target.

    The sheet covers barely a sixth of the frame's width, so at 1:1 the last few
    pixels of misalignment -- exactly the ones the tolerance is about -- are
    invisible.  Blowing the region up makes the final approach something you can
    actually see.
    """
    points = np.vstack([target, live]) if live is not None else np.asarray(target)
    margin = 14
    x0 = int(max(0, points[:, 0].min() - margin))
    y0 = int(max(0, points[:, 1].min() - margin))
    x1 = int(min(frame.shape[1], points[:, 0].max() + margin))
    y1 = int(min(frame.shape[0], points[:, 1].max() + margin))
    if x1 - x0 < 10 or y1 - y0 < 10:
        return

    crop = cv2.resize(frame[y0:y1, x0:x1], None, fx=scale, fy=scale,
                      interpolation=cv2.INTER_NEAREST)
    shift = np.array([x0, y0], dtype=float)
    draw_ghost(crop, (np.asarray(target) - shift) * scale, divisions, alpha=0.5)
    if live is not None:
        moved = (np.asarray(live) - shift) * scale
        cv2.polylines(crop, [moved.astype(np.int32)], True, colour, 2, cv2.LINE_AA)

    # Shrink to fit the corner it is pasted into, keeping the aspect ratio.
    box_w, box_h = 250, 190
    ratio = min(box_w / crop.shape[1], box_h / crop.shape[0], 1.0)
    crop = cv2.resize(crop, (max(1, int(crop.shape[1] * ratio)),
                             max(1, int(crop.shape[0] * ratio))))
    height, width = crop.shape[:2]
    px, py = panel.shape[1] - width - 8, panel.shape[0] - height - 8
    panel[py:py + height, px:px + width] = crop
    cv2.rectangle(panel, (px - 2, py - 2), (px + width + 1, py + height + 1), colour, 2)
    cv2.putText(panel, f'x{scale}', (px + 4, py + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1, cv2.LINE_AA)


def render_panel(frame, side, reference, corners, measured, active, action, divisions):
    if frame is None:
        panel = np.zeros((360, 640, 3), np.uint8)
        cv2.putText(panel, f'no frame from {side}', (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, YELLOW, 2, cv2.LINE_AA)
        return panel

    panel = frame.copy()
    if reference is not None:
        draw_ghost(panel, reference[side], divisions)
    if corners is not None:
        colour = GREEN if (measured and measured['verdict'] == 'OK') else RED
        cv2.polylines(panel, [corners.astype(np.int32)], True, colour, 2, cv2.LINE_AA)
        if reference is not None:
            # Arrows run from where the sheet is now to where it belongs.  The
            # other way round shows where it drifted from, which reads as the
            # opposite instruction and is exactly the wrong thing to follow.
            for start, end in zip(corners, reference[side]):
                if np.linalg.norm(end - start) > 1.5:
                    cv2.arrowedLine(panel, tuple(start.astype(int)), tuple(end.astype(int)),
                                    colour, 2, cv2.LINE_AA, tipLength=0.35)
            centre_now = corners.mean(axis=0)
            centre_target = reference[side].mean(axis=0)
            if np.linalg.norm(centre_target - centre_now) > 3.0:
                cv2.arrowedLine(panel, tuple(centre_now.astype(int)),
                                tuple(centre_target.astype(int)), colour, 4, cv2.LINE_AA,
                                tipLength=0.25)

    if not active:
        dim(panel, 0, 0, panel.shape[1], panel.shape[0], 0.55)

    verdict = 'NO SHEET' if measured is None else measured['verdict']
    colour = {'OK': GREEN, 'ADJUST': RED}.get(verdict, YELLOW)
    cv2.rectangle(panel, (0, 0), (panel.shape[1] - 1, panel.shape[0] - 1),
                  colour if active else GREY, 6 if active else 2)

    if active and reference is not None:
        draw_inset(panel, frame, reference[side], corners, divisions, colour)

    # The readout sits over live video, so the backdrop has to be dark enough
    # that thin coloured text stays legible against a bright floor.
    dim(panel, 6, 6, 320, 126, 0.82)
    bright = {'OK': (60, 255, 60), 'ADJUST': (80, 80, 255)}.get(verdict, YELLOW)
    header = f'{side.upper()}  {verdict}' + ('' if active else '  (done)')
    cv2.putText(panel, header, (14, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, bright, 2, cv2.LINE_AA)
    cv2.putText(panel, action if active else 'waiting for step 1', (14, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, WHITE if active else GREY, 1, cv2.LINE_AA)
    if measured is not None:
        cv2.putText(panel, f'off {measured["max_px"]:5.1f} px    scale {measured["scale"]:.3f}',
                    (14, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)
        if measured['hints'] and active:
            cv2.putText(panel, ' + '.join(measured['hints']), (14, 118),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, bright, 2, cv2.LINE_AA)
    return panel


def render(frames, reference, corners, results, step_index, message):
    panels = []
    for index, (side, _, action) in enumerate(STEPS):
        panels.append(render_panel(
            frames.get(side), side, reference, corners.get(side),
            results.get(side), index == step_index, action,
            (reference or {}).get(f'{side}_divisions', 8),
        ))
    stacked = np.hstack(panels)

    bar = np.zeros((64, stacked.shape[1], 3), np.uint8)
    side, moves, action = STEPS[step_index]
    cv2.putText(bar, f'STEP {step_index + 1}/2 - {moves}: {action}, watching the {side} camera',
                (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2, cv2.LINE_AA)
    cv2.putText(bar, message or 's=save reference   1/2=jump to step   q=quit',
                (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 1, cv2.LINE_AA)
    return np.vstack([stacked, bar])


# ------------------------------------------------------------------------ main

def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--reference', default=DEFAULT_REFERENCE)
    parser.add_argument('--board-tolerance', type=float, default=3.0,
                        help='pixels the sheet may sit off in the low camera (default: 3)')
    parser.add_argument('--aim-tolerance', type=float, default=3.0,
                        help='pixels the sheet may sit off in the high camera (default: 3)')
    parser.add_argument('--frames', type=int, default=7,
                        help='frames to median-filter each measurement over (default: 7)')
    parser.add_argument('--save', action='store_true',
                        help='save both cameras current view as the reference and exit')
    parser.add_argument('--headless', action='store_true',
                        help='print one verdict per camera and exit')
    args = parser.parse_args()

    captures, error = open_cameras(args.width, args.height)
    if captures is None:
        print(error, file=sys.stderr)
        return 2
    for side, capture in captures.items():
        print(f'{side}: {int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
              f'{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}')

    try:
        if args.save:
            return do_save(captures, args)

        try:
            reference = load_reference(args.reference)
            print(f'Reference: {args.reference} (saved {reference["saved_at"]})')
        except FileNotFoundError:
            print(f'No reference at {args.reference}. Run with --save first.', file=sys.stderr)
            return 2
        except ValueError as exc:
            print(f'Reference unusable: {exc}', file=sys.stderr)
            return 2

        if args.headless:
            return do_headless(captures, args, reference)
        return do_window(captures, args, reference)
    finally:
        for capture in captures.values():
            capture.release()
        cv2.destroyAllWindows()


def collect(captures, frames_wanted, warmup=5):
    for _ in range(warmup):
        read_frames(captures)
    history = {'high': [], 'low': []}
    squares = {'high': None, 'low': None}
    frames = {}
    for _ in range(frames_wanted):
        frames = read_frames(captures)
        for side, frame in frames.items():
            found = measure_side(frame)
            if found is not None:
                history[side].append(found['corners'])
                squares[side] = found['squares']
    corners = {side: median_corners(seen) for side, seen in history.items()}
    return corners, squares, frames


def do_save(captures, args):
    corners, squares, _ = collect(captures, args.frames)
    missing = [side for side, value in corners.items() if value is None]
    if missing:
        print(f'Sheet not found in: {", ".join(missing)}. Both cameras must see it.',
              file=sys.stderr)
        return 2
    save_reference(args.reference, corners, squares)
    print(f'Saved reference to {Path(args.reference).expanduser()}')
    for side, value in corners.items():
        centre = value.mean(axis=0)
        print(f'  {side}: centre ({centre[0]:.1f}, {centre[1]:.1f})')
    return 0


def do_headless(captures, args, reference):
    corners, _, _ = collect(captures, args.frames)
    tolerances = {'low': args.board_tolerance, 'high': args.aim_tolerance}
    results = {side: evaluate(reference, corners[side], side, tolerances[side])
               for side in ('high', 'low')}
    if any(value is None for value in results.values()):
        for index in range(len(STEPS)):
            print(summarise(index, results))
        print('Could not measure both cameras.', file=sys.stderr)
        return 2

    for index in range(len(STEPS)):
        print(summarise(index, results))
    return 0 if all(value['verdict'] == 'OK' for value in results.values()) else 1


def do_window(captures, args, reference):
    window = 'camera alignment'
    tolerances = {'low': args.board_tolerance, 'high': args.aim_tolerance}
    history = {'high': [], 'low': []}
    squares = {'high': None, 'low': None}
    step_index = 0
    message = ''
    last_report = 0.0

    while True:
        frames = read_frames(captures)
        if all(frame is None for frame in frames.values()):
            time.sleep(0.05)
            continue

        corners = {}
        for side, frame in frames.items():
            found = measure_side(frame)
            if found is None:
                history[side].clear()
            else:
                history[side].append(found['corners'])
                squares[side] = found['squares']
            del history[side][:-args.frames]
            corners[side] = median_corners(history[side]) if history[side] else None

        results = {side: evaluate(reference, corners[side], side, tolerances[side])
                   for side in ('high', 'low')}

        # Step 1 has to be finished before step 2 means anything: aiming the
        # camera at a sheet that is in the wrong place just moves the error.
        if step_index == 0 and results['low'] and results['low']['verdict'] == 'OK':
            step_index = 1
        elif step_index == 1 and results['low'] and results['low']['verdict'] != 'OK':
            step_index = 0
            message = 'the sheet moved - fix step 1 first'

        cv2.imshow(window, render(frames, reference, corners, results, step_index, message))

        if time.monotonic() - last_report > 1.0:
            print(summarise(step_index, results))
            last_report = time.monotonic()

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            return 0
        if key == ord('1'):
            step_index, message = 0, ''
        elif key == ord('2'):
            step_index, message = 1, ''
        elif key == ord('s'):
            if any(value is None for value in corners.values()):
                message = 'both cameras must see the sheet to save'
                continue
            save_reference(args.reference, corners, squares)
            reference = load_reference(args.reference)
            message = 'reference saved'
            print(f'Saved reference to {Path(args.reference).expanduser()}')


if __name__ == '__main__':
    sys.exit(main())
