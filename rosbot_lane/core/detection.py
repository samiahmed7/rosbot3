"""Lane line detection using Hough transform."""
import cv2
import numpy as np
from typing import Tuple, Optional, List

# Type alias for line: (slope, intercept) where x = slope * y + intercept
Line = Tuple[float, float]


def fit_lane_lines(
    binary: np.ndarray,
    img_width: int,
    hough_threshold: int = 20,
    min_line_length: int = 30,
    max_line_gap: int = 15,
    min_slope: float = 0.3,
    min_segments: int = 1
) -> Tuple[Optional[Line], Optional[Line]]:
    """
    Detect CL and RL using Hough transform.

    Returns (cl_line, rl_line) where each is (slope, intercept) or None.
    """
    segments = _get_hough_segments(binary, hough_threshold, min_line_length, max_line_gap)

    cl_line = detect_cl(binary, segments, img_width, min_slope, min_segments)
    rl_line = detect_rl(binary, segments, img_width, min_slope, min_segments)

    return cl_line, rl_line


def _get_hough_segments(
    binary: np.ndarray,
    hough_threshold: int,
    min_line_length: int,
    max_line_gap: int
) -> Optional[np.ndarray]:
    """Run HoughLinesP and return raw segments."""
    return cv2.HoughLinesP(
        binary,
        rho=1,
        theta=np.pi / 180,
        threshold=hough_threshold,
        minLineLength=min_line_length,
        maxLineGap=max_line_gap
    )


def detect_rl(
    binary: np.ndarray,
    segments: Optional[np.ndarray],
    img_width: int,
    min_slope: float = 0.3,
    min_segments: int = 1
) -> Optional[Line]:
    """
    Detect right lane line (solid).

    Returns (slope, intercept) or None.
    """
    if segments is None:
        return None

    h, w = binary.shape
    cx = img_width / 2.0

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
            candidates.append((bottom_x, slope, intercept))

    return _pick_best_line(candidates, closest_to='left', min_segments=min_segments)


def detect_cl(
    binary: np.ndarray,
    segments: Optional[np.ndarray],
    img_width: int,
    min_slope: float = 0.3,
    min_segments: int = 1
) -> Optional[Line]:
    """
    Detect centre lane line (dashed).

    Returns (slope, intercept) or None.
    """
    if segments is None:
        return None

    h, w = binary.shape
    cx = img_width / 2.0

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
            candidates.append((bottom_x, slope, intercept))

    return _pick_best_line(candidates, closest_to='right', min_segments=min_segments)


def _pick_best_line(
    candidates: List[Tuple[float, float, float]],
    closest_to: str,
    min_segments: int
) -> Optional[Line]:
    """Pick best line from candidates and average nearby ones."""
    if not candidates:
        return None

    if closest_to == 'right':
        candidates.sort(key=lambda c: c[0], reverse=True)
    else:
        candidates.sort(key=lambda c: c[0])

    best_x = candidates[0][0]
    group = [(s, i) for (bx, s, i) in candidates if abs(bx - best_x) < 20]

    if len(group) < min_segments:
        return None

    avg_slope = float(np.mean([p[0] for p in group]))
    avg_intercept = float(np.mean([p[1] for p in group]))
    return (avg_slope, avg_intercept)


def apply_corridor_mask(
    binary: np.ndarray,
    cl_line: Optional[Line],
    rl_line: Optional[Line],
    corridor_px: int
) -> np.ndarray:
    """Mask out pixels outside corridor around CL and RL."""
    if cl_line is None or rl_line is None:
        return binary

    h, w = binary.shape
    corridor = np.zeros_like(binary)

    for y in range(h):
        cl_x = int(cl_line[0] * y + cl_line[1])
        rl_x = int(rl_line[0] * y + rl_line[1])
        left_start = max(0, cl_x - corridor_px)
        right_end = min(w, rl_x + corridor_px)
        corridor[y, left_start:right_end] = 255

    return cv2.bitwise_and(binary, corridor)


def eval_line_at_y(line: Line, y: float) -> float:
    """Evaluate x position of line at given y."""
    return line[0] * y + line[1]
