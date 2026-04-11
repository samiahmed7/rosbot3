"""Lane geometry calculations."""

import numpy as np
from typing import Tuple, Optional


def compute_lane_error(
    cl_x: Optional[float],
    rl_x: Optional[float],
    img_width: int,
    lane_width_px: float
) -> Tuple[float, float]:
    """
    Compute normalised lateral error.
    
    Returns (error, lane_centre_x).
    Positive error = robot is left of lane centre.
    """
    if cl_x is not None and rl_x is not None:
        lane_centre_x = (cl_x + rl_x) / 2.0
    elif rl_x is not None:
        lane_centre_x = rl_x - lane_width_px / 2.0
    elif cl_x is not None:
        lane_centre_x = cl_x + lane_width_px / 2.0
    else:
        return 0.0, img_width / 2.0

    error = (img_width / 2.0 - lane_centre_x) / (img_width / 2.0)
    return float(np.clip(error, -1.0, 1.0)), float(lane_centre_x)
