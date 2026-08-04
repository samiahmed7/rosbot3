"""Lane line detection using Hough transform."""
import cv2
import numpy as np
from typing import Tuple, Optional

# Type alias for polynomial: [A, B, C] where x = Ay² + By + C
Poly = np.ndarray


def fit_lane_lines(
    binary: np.ndarray,
    img_width: int,
    hough_threshold: int = 20,
    min_line_length: int = 30,
    max_line_gap: int = 15,
    min_slope: float = 0.3,
    min_segments: int = 1,
    min_line_gap_px: int = 80
) -> Tuple[Optional[Poly], Optional[Poly]]:
    """
    Detect CL and RL using Hough transform.

    Returns (cl_line, rl_line) where each is [A, B, C] (x = Ay² + By + C) or None.
    """
    segments = _get_hough_segments(binary, hough_threshold, min_line_length, max_line_gap)

    cl_line = detect_cl(binary, segments, img_width, min_slope, min_segments)
    rl_line = detect_rl(binary, segments, img_width, min_slope, min_segments)

    if cl_line is not None and rl_line is not None:
        # On a sharp curve, the single visible boundary can cross from one
        # side of frame-center to the other within the strip — detect_cl
        # and detect_rl then both seed onto the *same* physical line and
        # each report a "detection". Averaging two copies of one line into
        # a fake "lane center" produces a large, wrong steering error, not
        # a real one. If the two fits are implausibly close together (well
        # under a real lane width apart), collapse them into one line
        # instead of trusting both as independent boundaries.
        h, w = binary.shape
        cx = img_width / 2.0
        cl_bot = cl_line[0] * h**2 + cl_line[1] * h + cl_line[2]
        rl_bot = rl_line[0] * h**2 + rl_line[1] * h + rl_line[2]
        if abs(rl_bot - cl_bot) < min_line_gap_px:
            merged = (cl_line + rl_line) / 2.0
            merged_bot = merged[0] * h**2 + merged[1] * h + merged[2]
            if merged_bot < cx:
                cl_line, rl_line = merged, None
            else:
                cl_line, rl_line = None, merged

    return cl_line, rl_line


def _get_hough_segments(
    binary: np.ndarray,
    hough_threshold: int,
    min_line_length: int,
    max_line_gap: int
) -> Optional[np.ndarray]:
    """Run HoughLinesP and return raw segments."""
    segments = cv2.HoughLinesP(
        binary,
        rho=1,
        theta=np.pi / 180,
        threshold=hough_threshold,
        minLineLength=min_line_length,
        maxLineGap=max_line_gap
    )
    return segments

def _sliding_window(
    binary: np.ndarray,
    start_x: int,
    n_windows: int = 20,
    margin: int = 80,
    min_pixels_recenter: int = 30,
    min_pixels_to_fit: int = 50
) -> Optional[Poly]:
    """
    Sliding window search from start_x, bottom to top.

    Returns polynomial [A, B, C] or None if not enough pixels.
    """
    h, w = binary.shape
    nzy, nzx = binary.nonzero()

    current_x = start_x
    window_h = h // n_windows
    pixel_indices = []

    for win in range(n_windows):
        y_lo = h - (win + 1) * window_h
        y_hi = h - win * window_h
        x_lo = max(0, current_x - margin)
        x_hi = min(w, current_x + margin)

        inds = np.where(
            (nzy >= y_lo) & (nzy < y_hi) &
            (nzx >= x_lo) & (nzx < x_hi)
        )[0]

        pixel_indices.append(inds)

        if len(inds) > min_pixels_recenter:
            current_x = int(np.mean(nzx[inds]))

    all_inds = np.concatenate(pixel_indices)

    if len(all_inds) < min_pixels_to_fit:
        return None

    # Fit polynomial: x = Ay² + By + C
    poly = np.polyfit(nzy[all_inds], nzx[all_inds], 2)

    # Sanity check: reject fits that extrapolate to a physically implausible
    # position at the bottom of the strip (the point actually used for
    # steering). White wall pixels outside the track pass the same HSV
    # segmentation as the real line, and a degree-2 fit pulled toward wall
    # noise can extrapolate to a position hundreds of pixels off-frame —
    # garbage that still gets treated as a confident detection otherwise.
    bottom_x = poly[0] * h**2 + poly[1] * h + poly[2]
    margin_px = 0.3 * w
    if bottom_x < -margin_px or bottom_x > w + margin_px:
        return None

    return poly

def detect_rl(
    binary: np.ndarray,
    segments: Optional[np.ndarray],
    img_width: int,
    min_slope: float = 0.3,
    min_segments: int = 1
) -> Optional[Poly]:
    """
    Detect right lane line (solid) using polynomial fitting.

    Returns [A, B, C] where x = Ay² + By + C, or None.
    """
    if segments is None:
        return None

    h, w = binary.shape
    cx = img_width / 2.0

    # Step 1: Use Hough segments to find starting position
    candidates = []
    for seg in segments:
        x1, y1, x2, y2 = seg[0]
        dy = float(y2 - y1)
        dx = float(x2 - x1)

        if abs(dx) < 1e-6 or abs(dy) < 1e-6:
            continue
        if abs(dy / dx) < min_slope:
            continue

        slope = dx / dy
        intercept = x1 - slope * y1
        bottom_x = slope * h + intercept

        # RL: right of center
        if bottom_x >= cx:
            candidates.append(bottom_x)
    if not candidates:
        return None

    # Step 2: Pick leftmost candidate (closest to center)
    candidates.sort()
    start_x = int(candidates[0])

    # Step 3: Sliding window + polynomial fit
    poly = _sliding_window(binary, start_x)
    return poly


    if not candidates:
        return None

    # Step 2: Pick leftmost candidate (closest to center)
    candidates.sort()
    start_x = int(candidates[0])

    # Step 3: Sliding window + polynomial fit
    return _sliding_window(binary, start_x)

def detect_cl(
    binary: np.ndarray,
    segments: Optional[np.ndarray],
    img_width: int,
    min_slope: float = 0.3,
    min_segments: int = 1
) -> Optional[Poly]:
    """
    Detect centre lane line (dashed) using polynomial fitting.

    Same approach as detect_rl: Hough segments only seed a starting x
    position, then a sliding-window pixel sweep does the actual fit — a
    dashed line's individual segments are short/gapped and frequently
    fail Hough's own length/vote thresholds directly, so unlike a solid
    line it can't be trusted to detect itself via Hough segments alone.

    Returns [A, B, C] where x = Ay² + By + C, or None.
    """
    if segments is None:
        return None

    h, w = binary.shape
    cx = img_width / 2.0

    # Step 1: Use Hough segments to find starting position
    candidates = []
    for seg in segments:
        x1, y1, x2, y2 = seg[0]
        dy = float(y2 - y1)
        dx = float(x2 - x1)

        if abs(dx) < 1e-6 or abs(dy) < 1e-6:
            continue
        if abs(dy / dx) < min_slope:
            continue

        slope = dx / dy
        intercept = x1 - slope * y1
        bottom_x = slope * h + intercept

        # CL: left of center
        if bottom_x < cx:
            candidates.append(bottom_x)

    if not candidates:
        return None

    # Step 2: Pick rightmost candidate (closest to center)
    candidates.sort(reverse=True)
    start_x = int(candidates[0])

    # Step 3: Sliding window + polynomial fit
    return _sliding_window(binary, start_x)


def apply_corridor_mask(
    binary: np.ndarray,
    cl_line: Optional[Poly],
    rl_line: Optional[Poly],
    corridor_px: int
) -> np.ndarray:
    """
    Mask out pixels outside a corridor around the previously-tracked
    line(s), so a new frame's search stays anchored near known-good
    history instead of freely reconsidering every white pixel in the
    strip (which is what lets an unrelated line — an intersection edge,
    a roundabout marking, the far lane boundary — hijack detection the
    moment it enters view). Applied independently per line: a line
    that's currently tracked keeps constraining search near itself even
    while the other line is lost, rather than requiring both.
    """
    if cl_line is None and rl_line is None:
        return binary

    h, w = binary.shape
    corridor = np.zeros_like(binary)

    for y in range(h):
        if cl_line is not None:
            cl_x = int(cl_line[0] * y**2 + cl_line[1] * y + cl_line[2])
        if rl_line is not None:
            rl_x = int(rl_line[0] * y**2 + rl_line[1] * y + rl_line[2])

        if cl_line is not None and rl_line is not None:
            # Keep the full lane interior between them, not just narrow
            # bands around each — the mat surface between the two lines
            # is expected to be clean anyway.
            left_start = max(0, cl_x - corridor_px)
            right_end = min(w, rl_x + corridor_px)
            corridor[y, left_start:right_end] = 255
        elif cl_line is not None:
            # Only CL tracked — protect its immediate left margin, but
            # leave everything from there rightward fully open so RL
            # (untracked, lives further right) can still be found. Only
            # excludes stuff even further left than CL itself.
            corridor[y, max(0, cl_x - corridor_px):w] = 255
        elif rl_line is not None:
            # Only RL tracked — mirror of the above: leave everything up
            # to RL's margin open so CL (untracked, lives further left,
            # e.g. a dashed line between dashes) can still be reacquired,
            # rather than getting masked out of existence entirely.
            corridor[y, 0:min(w, rl_x + corridor_px)] = 255

    return cv2.bitwise_and(binary, corridor)

def eval_poly_at_y(poly: Poly, y: float) -> float:
    """Evaluate x position of polynomial at given y."""
    return poly[0] * y**2 + poly[1] * y + poly[2]
