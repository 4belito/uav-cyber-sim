
from helpers.math import manhattan_distance
import numpy as np

# def in_same_quadrant(current,waypoint,target,eps=1):
#     x_curr, y_curr = current
#     x_target, y_target = target
#     x_wp, y_wp = waypoint
#     return (
#         [(x_curr - x_target)>-eps and (x_curr - x_wp) > -eps] or [(x_curr - x_target)<eps and (x_curr- x_wp) <eps] and # X direction check
#         [(y_curr - y_target)>-eps and (y_curr - y_wp) > -eps] or [(y_curr - y_target)<eps and (y_curr - x_wp) <eps]  # Y direction check
#     )
def in_same_orthant(current: np.ndarray, target: np.ndarray, waypoints: np.ndarray, dims=[0,1], eps=1) -> np.ndarray:
    """
    Vectorized function to check if waypoints are in the same quadrant as the target w.r.t. the current position.

    Parameters:
    - current: np.ndarray of shape (3,) (current position)
    - target: np.ndarray of shape (3,) (target position)
    - waypoints: np.ndarray of shape (N,3) (list of waypoints)
    - dims: list of int (dimensions to consider, default [0,1] for 2D checks)
    - eps: float (tolerance for quadrant checks)

    Returns:
    - np.ndarray of shape (N,) (boolean array indicating which waypoints are in the same quadrant)
    """
    # Extract only relevant dimensions
    current = current[dims]
    target = target[dims]
    waypoints = waypoints[:, dims]

    # Check if waypoints are in the same orthant
    target_rel =  current - target
    waypoints_rel=current - waypoints
    return np.all(((target_rel >= -eps) & (waypoints_rel >= -eps))| ((target_rel <= eps) & (waypoints_rel <= eps)), axis=1)


# def in_same_corredor(current,waypoint,eps=1):
#     x_way, y_way = waypoint
#     x_cur, y_cur = current
#     return abs(x_way -x_cur)<eps or abs(y_way - y_cur)<eps

def in_same_corridor(current: np.ndarray, waypoints: np.ndarray, eps=1,dims=[0,1]) -> np.ndarray:
    """
    Vectorized function to check if waypoints are in the same corridor as the current position.

    Parameters:
    - current: np.ndarray of shape (2,) (current position [x, y])
    - waypoints: np.ndarray of shape (N,2) (list of waypoints)
    - eps: float (tolerance for corridor check)

    Returns:
    - np.ndarray of shape (N,) (boolean array indicating which waypoints are in the same corridor)
    """
    # Compute absolute difference along x and y directions
    delta = np.abs(waypoints[:,dims] - current[dims])

    # Check if waypoints lie within the corridor (x or y within eps)
    return np.any(delta < eps, axis=1)


def find_best_waypoint(current, target, waypoints,eps=1,same_orth=False):
    # Filter points that share x or y with the target AND are in the correct quadrant
    if same_orth:
        same_quadrant=in_same_orthant(current, target,waypoints,eps=eps)
        waypoints= waypoints[same_quadrant]
    mask=in_same_corridor(current,waypoints,eps=eps)
    valid_waypoints = waypoints[mask]
    if valid_waypoints.shape[0] == 0:
        return None,None  # No valid moves available
    
    dist_to_curr = manhattan_distance(valid_waypoints, current)
    dist_to_target = manhattan_distance(valid_waypoints, target)
    best_i_valid = np.argmin(dist_to_curr + dist_to_target)
    best_i_original = np.where(mask)[0][best_i_valid]
    # Find the move that gets closest to the current position (gredy search)
    #best_move = min(valid_waypoints, key=lambda w: manhattan_distance(w, current)+manhattan_distance(w, target))
    return valid_waypoints[best_i_valid],np.delete(waypoints, best_i_original, axis=0) 

def find_path(start, target, waypoints,eps=1):
    path = [start]
    current = start
    while not np.array_equal(current, target):
        # same_quadrant=in_same_orthant(current, target,waypoints,eps=eps)
        # waypoints = waypoints[same_quadrant]
        next_waypoint,waypoints= find_best_waypoint(current, target, waypoints,eps=eps)
        if next_waypoint is None:
            break  # No valid path found
        path.append(next_waypoint)
        current = next_waypoint
    return np.stack(path,axis=0)