"""Lane geometry calculations."""

import numpy as np
from typing import Tuple, Optional


# RL/CL offset calibration (when robot centered, RL_bot = 605, img_center = 384).
# NOTE: that calibration point predates the img_width fix (768 -> 640, true
# center 384 -> 320) — both offsets below need re-verification against the
# corrected resolution, not just trusted as-is. CL_OFFSET_PX is a mirrored
# starting guess (not independently calibrated) since CL-only fallback is new.
RL_OFFSET_PX = 165.0
CL_OFFSET_PX = 165.0


def compute_lane_error_rl_only(
    rl_x: Optional[float],
    img_width: int
) -> Tuple[float, Optional[float]]:
    """
    Compute lane error using only RL (right lane).

    Args:
        rl_x: RL x-position at bottom of strip
        img_width: Image width in pixels

    Returns:
        (error, lane_center_x) - Error in [-1, 1], None if RL not detected.
    """
    if rl_x is None:
        return 0.0, None

    img_center = img_width / 2.0
    lane_center_x = rl_x - RL_OFFSET_PX
    error = (img_center - lane_center_x) / img_center
    return float(np.clip(error, -1.0, 1.0)), float(lane_center_x)


def compute_lane_error_cl_only(
    cl_x: Optional[float],
    img_width: int
) -> Tuple[float, Optional[float]]:
    """
    Compute lane error using only CL (center/left line) — mirror of
    compute_lane_error_rl_only, for when RL has swung out of view or into
    a dash-style gap (the CL/RL <-> dashed/solid mapping isn't fixed: which
    physical line lands left vs right of frame center can flip mid-turn).

    Args:
        cl_x: CL x-position at bottom of strip
        img_width: Image width in pixels

    Returns:
        (error, lane_center_x) - Error in [-1, 1], None if CL not detected.
    """
    if cl_x is None:
        return 0.0, None

    img_center = img_width / 2.0
    lane_center_x = cl_x + CL_OFFSET_PX
    error = (img_center - lane_center_x) / img_center
    return float(np.clip(error, -1.0, 1.0)), float(lane_center_x)


def compute_lane_error(
    cl_x: Optional[float],
    rl_x: Optional[float],
    img_width: int,
    lane_width_px: float
) -> Tuple[float, float]:
    """
    Compute lane error using both CL and RL (original method).
    """
    if cl_x is None or rl_x is None:
        return 0.0, img_width / 2.0

    lane_centre_x = (cl_x + rl_x) / 2.0
    error = (img_width / 2.0 - lane_centre_x) / (img_width / 2.0)
    return float(np.clip(error, -1.0, 1.0)), float(lane_centre_x)