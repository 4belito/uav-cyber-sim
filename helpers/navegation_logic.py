
from helpers.math import manhattan_distance

def in_same_quadrant(current,waypoint,target,eps=1):
    x_curr, y_curr = current
    x_target, y_target = target
    x_wp, y_wp = waypoint
    return (
        [(x_curr - x_target)>-eps and (x_curr - x_wp) > -eps] or [(x_curr - x_target)<eps and (x_curr- x_wp) <eps] and # X direction check
        [(y_curr - y_target)>-eps and (y_curr - y_wp) > -eps] or [(y_curr - y_target)<eps and (y_curr - x_wp) <eps]  # Y direction check
    )

def in_same_corredor(curr_pos,waypoint,eps=1):
    x_way, y_way = waypoint
    x_cur, y_cur = curr_pos
    return abs(x_way -x_cur)<eps or abs(y_way - y_cur)<eps


def find_best_waypoint(curr_pos, target_pos, waypoints,eps=1):
    # Filter points that share x or y with the target AND are in the correct quadrant
    valid_waypoints = [w for w in waypoints if in_same_corredor(curr_pos,w,eps=eps) and in_same_quadrant(curr_pos,w,target_pos,eps=eps)]

    if not valid_waypoints:
        return None  # No valid moves available

    # Find the move that gets closest to the current position (gredy search)
    best_move = min(valid_waypoints, key=lambda w: manhattan_distance(w, curr_pos)+manhattan_distance(w, target_pos))

    return best_move

def find_path(start, target, waypoints):
    path = [start]
    current = start

    while current!= target:
        next_waypoint = find_best_waypoint(current, target, waypoints)
        if next_waypoint is None:
            break  # No valid path found
        path.append(next_waypoint)
        waypoints.remove(next_waypoint)  # Remove the used waypoint
        current = next_waypoint

    return path