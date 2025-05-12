# type: ignore

from functools import partial

import numpy as np
from numpy.typing import NDArray
from pymavlink import mavutil

from helpers.change_coordinates import Position, global_switch_local_ned
from plan.core import Action, ActionNames, Step

TYPE_MASK = int(0b110111111000)
LOCAL_COORD = mavutil.mavlink.MAV_FRAME_LOCAL_NED


def get_local_position(conn: mavutil.mavfile):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    ## Check this to make blocking optional parameter
    msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.001)
    # This does not work. I'am not sure why
    # msg = conn.recv_match(type="LOCAL_POSITION_NED")
    if msg:
        return np.array(global_switch_local_ned((msg.x, msg.y, msg.z)))
    else:
        return None


def exec_go_local(
    conn: mavutil.mavfile,
    wp: Position,
):
    wp = global_switch_local_ned(wp)
    go_msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,
        conn.target_system,
        conn.target_component,
        LOCAL_COORD,
        TYPE_MASK,
        *wp,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    conn.mav.send(go_msg)


def check_reach_wp(
    conn: mavutil.mavfile,
    verbose: int,
    wp: np.ndarray = np.array([0, 0, 10]),
    wp_margin=0.5,
):
    """Check if the UAV has reached the target altitude within an acceptable margin."""
    pos = get_local_position(conn)
    if pos is not None:
        dist = np.linalg.norm(pos - wp)
        if verbose > 1:
            print(f"Vehicle {conn.target_system}:📍 Distance to target: {dist:.2f} m")
        answer = dist < wp_margin
    else:
        answer = False

    return answer, pos


## Make the action
def make_path(wps: np.ndarray = None, wp_margin: float = 0.5):
    go_local_action = Action(name=ActionNames.FLY, emoji="🛩️")
    if wps is None:
        return go_local_action  # Return empty action if no waypoints
    for wp in wps:
        go_local_action.add(make_go_to(wp, wp_margin))

    return go_local_action


def make_go_to(
    wp: np.ndarray = None,
    wp_margin: float = 0.5,
    cause_text="",
    target_pos: np.ndarray = None,
    is_improv: bool = False,
):
    if target_pos is None:
        target_pos = wp
    goto_step = Step(
        f"go to {cause_text} -> {fmt(wp)}",
        check_fn=partial(check_reach_wp, wp=wp, wp_margin=wp_margin),
        exec_fn=partial(exec_go_local, wp=wp),
        target_pos=target_pos,
        onair=True,
        is_improv=is_improv,
    )
    return goto_step


def fmt(wp: NDArray[np.float64]):
    return tuple(int(x) if float(x).is_integer() else round(float(x), 2) for x in wp)
