from helpers.math import manhattan_distance
import numpy as np


def in_same_orthant(
    current: np.ndarray, target: np.ndarray, waypoints: np.ndarray, dims=None, eps=1
) -> np.ndarray:
    """
    Vectorized function to check if waypoints are in the same quadrant as the target w.r.t. the current position.
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
    current: np.ndarray, waypoints: np.ndarray, eps=1, dims=None
) -> np.ndarray:
    """
    Vectorized function to check if waypoints are in the same corridor as the current position.
    """
    if dims is None:
        dims = [0, 1, 2]
    delta = np.abs(waypoints[:, dims] - current[dims])
    return np.sum(delta < eps, axis=1) >= 2


def delete(current: np.ndarray, waypoints: np.ndarray, eps=1, dims=None) -> np.ndarray:
    """
    Vectorized function to check if waypoints are in the same corridor as the current position.
    """
    if dims is None:
        dims = [0, 1]
    delta = np.abs(waypoints[:, dims] - current[dims])
    return np.any(delta < eps, axis=1)


def remove_wp(arr: np.ndarray, row: np.ndarray, eps: float = 1):
    """
    Removes rows from a 2D NumPy array that are within a distance <= eps from a given row.
    """
    # Compute Euclidean distance from each row to the target row
    distances = np.linalg.norm(arr - row, axis=1)

    # Keep only rows with distance > eps
    mask = distances > eps

    return arr[mask]


def adjust_one_significant_axis_toward_corridor(
    current: np.ndarray, waypoints: np.ndarray, eps: float = 1.0
) -> np.ndarray:
    """
    Adjusts the closest significant axis (above `eps` difference) to bring the drone closer to a valid corridor,
    modifying only one coordinate.
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


def get_valid_waypoints(current, target, waypoints, eps=1, same_orthant=False):
    # Filter points that share x or y with the target AND are in the correct quadrant
    waypoints = remove_wp(waypoints, current, eps=eps)
    if same_orthant:
        same_quadrant = in_same_orthant(current, target, waypoints, eps=eps)
        waypoints = waypoints[same_quadrant]
    mask = in_same_corridor(current, waypoints, eps=eps)
    return waypoints[mask]


def find_best_waypoint(current, target, valid_waypoints):
    dist_to_curr = manhattan_distance(valid_waypoints, current)
    dist_to_target = manhattan_distance(valid_waypoints, target)
    best_i_valid = np.argmin(dist_to_curr + dist_to_target)
    return valid_waypoints[best_i_valid]


def next_position(current, target, waypoints, eps, same_orthant=False):
    valid_waypoints = get_valid_waypoints(
        current, target, waypoints, eps, same_orthant=same_orthant
    )
    if valid_waypoints.shape[0] == 0:
        next_pos = adjust_one_significant_axis_toward_corridor(current, waypoints, eps)
    else:
        next_pos = find_best_waypoint(current, target, valid_waypoints)
    return next_pos


def find_path(start, target, waypoints, eps=1):
    path = [start]
    current = start
    while not np.array_equal(current, target):
        next_pos = next_position(current, target, waypoints, eps)
        current = next_pos
        path.append(current)
    return np.stack(path, axis=0)
