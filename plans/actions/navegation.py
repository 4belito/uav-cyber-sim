
from pymavlink import mavutil
import numpy as np
from functools import partial

from plans.planner import Step, Action
from helpers.change_coordinates import GLOBAL_switch_LOCAL_NED

TYPE_MASK= int(0b110111111000)
LOCAL_COORD= mavutil.mavlink.MAV_FRAME_LOCAL_NED

def exec_go_local(conn: mavutil.mavlink_connection, blocking=False, wp: np.ndarray = np.array([1, 1, 10])):
    wp=GLOBAL_switch_LOCAL_NED(*wp)
    go_msg=mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, conn.target_system, conn.target_component, LOCAL_COORD, TYPE_MASK, *wp, 0, 0, 0, 0, 0, 0, 0, 0)
    conn.mav.send(go_msg)


# 📡 Check if UAV reached the wp
def check_reach_wp(conn: mavutil.mavlink_connection, blocking=False, wp: np.ndarray = np.array([0, 0, 10]),wp_margin=0.5):
    """Check if the UAV has reached the target altitude within an acceptable margin."""
    msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=blocking, timeout=2)
    if not msg:
        return False  # No valid position data received

    # Convert coordinates
    pos = GLOBAL_switch_LOCAL_NED(msg.x, msg.y, msg.z)

    # Compute distance to target waypoint
    dist = np.linalg.norm(pos - wp)
    print(f"📍 Distance to target: {dist:.2f} m")

    # Check if the UAV has reached the waypoint within the margin
    if dist > wp_margin:
        return False

    return True  # UAV has reached the waypoint


def make_path(wps:np.ndarray,wp_margin:float=0.5):
    go_local_action = Action("Go local")
    for wp in wps:
        go_local_action.add(Step(f"go to -> {tuple(wp)}",
                                check_fn=partial(check_reach_wp,wp=wp,wp_margin=wp_margin),
                                exec_fn=partial(exec_go_local,wp=wp)))

    return go_local_action