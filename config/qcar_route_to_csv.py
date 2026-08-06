#!/usr/bin/env python3
"""Convert the QCar's route to the CSV the ROSbot's follower reads.

Both vehicles share QCar 2's Cartographer map, so they can share its route.
The QCar's file is already spline-smoothed and resampled at 0.05 m by
smooth_path.py -- this only changes the container, it does NOT re-smooth.
Running it through config/smooth.py again would round the corners twice.

    python3 config/qcar_route_to_csv.py \
        --npy /tmp/rosbot_logs/my_route_loop.npy \
        --out config/shared_route.csv

Why the QCar's recording and not the ROSbot's: the QCar is Ackermann with a
minimum turning radius, the ROSbot is differential drive. Anything the QCar
can drive, the ROSbot can follow. The reverse is not true.

The checks below mirror Trajectory._detect_segments in
rosbot_lane/core/trajectory.py, which splits a route into segments wherever
consecutive travel directions have dot < -0.5 (a >120 deg reversal) and
drives to each flip point in turn. A route recorded by a vehicle that never
reverses should produce exactly one forward segment; anything else means the
ROSbot would try to reverse mid-loop.
"""

import argparse
import csv
import math
import sys

import numpy as np


def analyse(xy, theta):
    """Report what ROSbot's Trajectory class will make of this route."""
    d = np.diff(xy, axis=0)
    step = np.hypot(d[:, 0], d[:, 1])
    ok = step > 1e-3
    dirs = d[ok] / step[ok, None]

    dots = np.sum(dirs[:-1] * dirs[1:], axis=1)
    flips = int(np.sum(dots < -0.5))

    travel0 = math.atan2(dirs[0][1], dirs[0][0])
    diff = abs(math.atan2(math.sin(travel0 - theta[0]),
                          math.cos(travel0 - theta[0])))
    initial_reverse = diff > 2.0

    closed = float(np.hypot(*(xy[0] - xy[-1])))

    n = len(xy)
    d = np.hypot(xy[:, 0] - xy[0, 0], xy[:, 1] - xy[0, 1])
    lap = next((i for i in range(20, n) if d[i] < 0.15
                and d[i] <= d[i - 1] and d[i] <= d[min(n - 1, i + 1)]), None)

    # Fraction of waypoints that have a non-adjacent waypoint close enough to
    # confuse find_closest_waypoint's search.
    sep = np.abs(np.arange(n)[:, None] - np.arange(n)[None, :])
    dist = np.hypot(xy[:, None, 0] - xy[None, :, 0], xy[:, None, 1] - xy[None, :, 1])
    far_branch = np.where(np.minimum(sep, n - sep) > 40, dist, np.inf).min(axis=1)

    return {
        "points": n,
        "length_m": float(step.sum()),
        "spacing_mean": float(step.mean()),
        "spacing_min": float(step.min()),
        "spacing_max": float(step.max()),
        "flips": flips,
        "min_dot": float(dots.min()),
        "initial_reverse": bool(initial_reverse),
        "heading_err_deg": math.degrees(diff),
        "closure_gap_m": closed,
        "lap_points": lap,
        "ambiguous_frac": float((far_branch < 0.60).mean()),
        "min_self_approach_m": float(far_branch.min()),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--npy", required=True, help="QCar my_route_loop.npy")
    ap.add_argument("--out", required=True, help="CSV to write (x,y,theta)")
    ap.add_argument("--single-lap", action="store_true",
                    help="truncate at the first return to the start")
    ap.add_argument("--force", action="store_true",
                    help="write even if the route looks unusable")
    args = ap.parse_args()

    route = np.load(args.npy)
    if route.ndim != 2 or route.shape[1] < 3:
        raise SystemExit(f"expected (N,>=3), got {route.shape}")

    xy = route[:, :2].astype(float)
    theta = route[:, 2].astype(float)
    # The recorder stores unwrapped yaw (values beyond +/-pi are normal);
    # the ROSbot normalises internally, but write it wrapped so the file is
    # readable and comparisons against live poses are obvious.
    theta = np.arctan2(np.sin(theta), np.cos(theta))

    info = analyse(xy, theta)

    # The QCar's recording is several laps of one circuit. The ROSbot's
    # Trajectory has no lap concept -- it drives waypoint 0 to the last one,
    # and its searches assume the route never revisits itself -- so keep one.
    if args.single_lap:
        if not info["lap_points"]:
            raise SystemExit("--single-lap: route never returns to its start")
        print(f"  --single-lap  : keeping {info['lap_points']} of {len(xy)} points\n")
        xy = xy[:info["lap_points"]]
        theta = theta[:info["lap_points"]]
        info = analyse(xy, theta)

    print(f"  points        : {info['points']}")
    print(f"  length        : {info['length_m']:.2f} m")
    print(f"  spacing       : mean {info['spacing_mean']:.4f} m "
          f"(min {info['spacing_min']:.4f}, max {info['spacing_max']:.4f})")
    print(f"  loop closure  : {info['closure_gap_m']:.3f} m between last and first")
    print(f"  bbox          : x {xy[:,0].min():+.2f}..{xy[:,0].max():+.2f}   "
          f"y {xy[:,1].min():+.2f}..{xy[:,1].max():+.2f}")
    print()
    print("  ROSbot Trajectory._detect_segments will see:")
    print(f"    direction flips (dot < -0.5) : {info['flips']}  "
          f"(min dot {info['min_dot']:+.3f})")
    print(f"    initial segment reversed     : {info['initial_reverse']}  "
          f"(travel vs heading {info['heading_err_deg']:.1f} deg)")
    print(f"    laps                         : "
          f"{'first return at wp %d' % info['lap_points'] if info['lap_points'] else 'none detected'}")
    print(f"    self-approach                : "
          f"{100 * info['ambiguous_frac']:.0f}% of waypoints have a far-branch "
          f"neighbour < 0.60 m (closest {info['min_self_approach_m']:.3f} m)")

    problems = []
    if info["ambiguous_frac"] > 0.05:
        problems.append(
            f"{100 * info['ambiguous_frac']:.0f}% of waypoints sit within 0.60 m "
            "of a different part of the route -- find_closest_waypoint will "
            "snap to the wrong branch and the lookahead will jump. Pass "
            "--single-lap")
    if info["flips"]:
        problems.append(
            f"{info['flips']} direction flip(s): the ROSbot will split this "
            "into segments and try to reverse at each one")
    if info["initial_reverse"]:
        problems.append(
            "first segment reads as REVERSE: recorded heading disagrees with "
            "travel direction, so the ROSbot would drive it backwards")
    if info["spacing_max"] > 3 * info["spacing_mean"]:
        problems.append(
            f"spacing jumps to {info['spacing_max']:.3f} m "
            f"(mean {info['spacing_mean']:.3f}) -- a gap in the recording")

    if problems:
        print()
        for p in problems:
            print(f"  WARNING: {p}")
        if not args.force:
            print("\n  not written. re-run with --force to write anyway.")
            return 1

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "theta"])
        for (x, y), t in zip(xy, theta):
            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{t:.6f}"])

    print(f"\n  wrote {args.out} ({info['points']} rows)")
    print("  NOTE: point this at BOTH the follower and the V2V broadcaster")
    print("        (-p trajectory_csv:=...), or the broadcaster predicts")
    print("        along a route in a different frame than its own pose.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
