from helpers.math import manhattan_distance
import numpy as np


def in_same_orthant(
    current: np.ndarray, target: np.ndarray, waypoints: np.ndarray, dims=[0, 1], eps=1
) -> np.ndarray:
    """
    Vectorized function to check if waypoints are in the same quadrant as the target w.r.t. the current position.
    """
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
    current: np.ndarray, waypoints: np.ndarray, eps=1, dims=[0, 1, 2]
) -> np.ndarray:
    """
    Vectorized function to check if waypoints are in the same corridor as the current position.
    """
    delta = np.abs(waypoints[:, dims] - current[dims])
    return np.sum(delta < eps, axis=1) >= 2


def delete(
    current: np.ndarray, waypoints: np.ndarray, eps=1, dims=[0, 1]
) -> np.ndarray:
    """
    Vectorized function to check if waypoints are in the same corridor as the current position.
    """
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


def find_best_waypoint(current, target, waypoints, eps=1, same_orthant=False):
    # Filter points that share x or y with the target AND are in the correct quadrant
    waypoints = remove_wp(waypoints, current, eps=eps)
    if same_orthant:
        same_quadrant = in_same_orthant(current, target, waypoints, eps=eps)
        waypoints = waypoints[same_quadrant]
    mask = in_same_corridor(current, waypoints, eps=eps)
    valid_waypoints = waypoints[mask]

    if valid_waypoints.shape[0] == 0:
        return None, None  # No valid moves available

    dist_to_curr = manhattan_distance(valid_waypoints, current)
    dist_to_target = manhattan_distance(valid_waypoints, target)
    best_i_valid = np.argmin(dist_to_curr + dist_to_target)
    return valid_waypoints[best_i_valid]


def find_path(start, target, waypoints, eps=1):
    path = [start]
    current = start
    while not np.array_equal(current, target):
        next_waypoint = find_best_waypoint(current, target, waypoints, eps=eps)
        if next_waypoint is None:
            break  # No valid path found
        current = next_waypoint
        path.append(next_waypoint)
    return np.stack(path, axis=0)
