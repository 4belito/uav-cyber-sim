"""
Waypoint filtering and navigation logic for selecting movement steps based on spatial
constraints.

This module includes utilities to:
- Determine geometric relationships between waypoints (orthants, corridors, proximity)
- Filter and adjust waypoint candidates
- Build stepwise paths from a start to target position using discrete axis-aligned
movements
"""

from typing import List

import numpy as np
from numpy.typing import NDArray

from helpers.math import manhattan_distance


def in_same_orthant(
    current: NDArray[np.float64],
    target: NDArray[np.float64],
    waypoints: NDArray[np.float64],
    dims: None | List[int] = None,
    eps: float = 1.0,
) -> NDArray[np.bool_]:
    """
    Return a mask for waypoints in the same orthant as the target w.r.t. the current
    position.
    """
    if dims is None:
        dims = [0, 1]

    # Extract only relevant dimensions
    current = current[dims]
    target = target[dims]
    waypoints = waypoints[:, dims]

    # Check if waypoints are in the same orthant
    target_rel = current - target
    waypoints_rel = current - waypoints
    return np.all(
        ((target_rel >= -eps) & (waypoints_rel >= -eps))
        | ((target_rel <= eps) & (waypoints_rel <= eps)),
        axis=1,
    )


def in_same_corridor(
    current: NDArray[np.float64],
    waypoints: NDArray[np.float64],
    eps: float = 1.0,
    dims: None | List[int] = None,
) -> NDArray[np.bool_]:
    """
    Return a mask for waypoints that match the current position along at
    least two axes (within eps).
    """
    if dims is None:
        dims = [0, 1, 2]
    delta = np.abs(waypoints[:, dims] - current[dims])
    return np.sum(delta < eps, axis=1) >= 2


def is_close_to(
    current: NDArray[np.float64],
    waypoints: NDArray[np.float64],
    eps: float = 1.0,
    dims: None | List[int] = None,
) -> NDArray[np.bool_]:
    """
    Return a boolean mask for waypoints close to the current point along given
    dimensions.
    """
    if dims is None:
        dims = [0, 1]
    delta = np.abs(waypoints[:, dims] - current[dims])
    return np.any(delta < eps, axis=1)


def remove_wp(
    arr: NDArray[np.float64], row: NDArray[np.float64], eps: float = 1.0
) -> NDArray[np.float64]:
    """
    Remove rows from a 2D array that are within eps (Euclidean distance) of a given
    row.
    """
    # Compute Euclidean distance from each row to the target row
    distances = np.linalg.norm(arr - row, axis=1)

    # Keep only rows with distance > eps
    mask = distances > eps

    return arr[mask]


def adjust_one_significant_axis_toward_corridor(
    current: NDArray[np.float64], waypoints: NDArray[np.float64], eps: float = 1.0
) -> NDArray[np.float64]:
    """
    Adjust one significant axis (difference > eps) to bring the drone closer to a valid
    corridor.
    Only one coordinate is modified.
    """
    diffs = np.abs(waypoints - current)  # shape (N, 3)
    dists = np.sum(diffs, axis=1)  # total Manhattan distance to each waypoint

    j = np.argmin(dists)
    closest_wp = waypoints[j]
    axis_diffs = diffs[j]

    # Get axis indices sorted by increasing difference
    axis_order = np.argsort(axis_diffs)

    for axis in axis_order:
        if axis_diffs[axis] >= eps:
            new_pos = np.array(current)
            new_pos[axis] = closest_wp[axis]
            return new_pos

    # If all axes are already too close, return current (no need to adjust)
    return np.array(current)


def get_valid_waypoints(
    current: NDArray[np.float64],
    target: NDArray[np.float64],
    waypoints: NDArray[np.float64],
    eps: float = 1.0,
    same_orthant: bool = False,
) -> NDArray[np.float64]:
    """
    Filter points that share x or y with the target AND are in the correct
    quadrant.
    """
    waypoints = remove_wp(waypoints, current, eps=eps)
    if same_orthant:
        same_quadrant_mask = in_same_orthant(current, target, waypoints, eps=eps)
        waypoints = waypoints[same_quadrant_mask]
    mask = in_same_corridor(current, waypoints, eps=eps)
    return waypoints[mask]


def find_best_waypoint(
    current: NDArray[np.float64],
    target: NDArray[np.float64],
    valid_waypoints: NDArray[np.float64],
) -> NDArray[np.float64]:
    """
    Return the waypoint closest to both current and target positions using
    Manhattan distance.
    """
    dist_to_curr = manhattan_distance(valid_waypoints, current)
    dist_to_target = manhattan_distance(valid_waypoints, target)
    best_i_valid = np.argmin(dist_to_curr + dist_to_target)
    return valid_waypoints[best_i_valid]


def next_position(
    current: NDArray[np.float64],
    target: NDArray[np.float64],
    waypoints: NDArray[np.float64],
    eps: float = 1.0,
    same_orthant: bool = False,
) -> NDArray[np.float64]:
    """Get the next best position along the path from current to target."""
    valid_waypoints = get_valid_waypoints(
        current, target, waypoints, eps, same_orthant=same_orthant
    )
    if valid_waypoints.shape[0] == 0:
        next_pos = adjust_one_significant_axis_toward_corridor(current, waypoints, eps)
    else:
        next_pos = find_best_waypoint(current, target, valid_waypoints)
    return next_pos


def find_path(
    start: NDArray[np.float64],
    target: NDArray[np.float64],
    waypoints: NDArray[np.float64],
    eps: float = 1.0,
):
    """Build a path from start to target using discrete valid steps."""
    path = [start]
    current = start
    while not np.array_equal(current, target):
        next_pos = next_position(current, target, waypoints, eps)
        current = next_pos
        path.append(current)
    return np.stack(path, axis=0)
