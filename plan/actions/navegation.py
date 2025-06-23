"""
Defines logic for navigating to local NED waypoints using MAVLink.

Includes:
- Functions to command and check UAV movement in local coordinates.
- Construction of Step and Action objects to integrate into mission plans.
"""

import math
from functools import partial

import numpy as np
from numpy.typing import NDArray
from pymavlink import mavutil

from helpers.change_coordinates import Position, global_switch_local_ned
from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import Frame, MsgID
from mavlink.util import ask_msg, stop_msg
from plan.core import Action, ActionNames, Step


def make_go_to(
    wp: Position,
    wp_margin: float = 0.5,
    cause_text: str = "",
    target_pos: Position | None = None,
    is_improv: bool = False,
) -> Step:
    """Build a Step that moves the UAV to a specific waypoint."""
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


## Make the action
def make_path(
    wps: NDArray[np.float64] | None = None, wp_margin: float = 0.5
) -> Action[Step]:
    """Create a FLY action composed of multiple go-to waypoint steps."""
    go_local_action = Action[Step](name=ActionNames.FLY, emoji="🛩️")
    if wps is None:
        return go_local_action  # Return empty action if no waypoints
    for wp in wps:
        go_local_action.add(make_go_to(wp, wp_margin))

    return go_local_action


def get_local_position(conn: MAVConnection):
    """Request and return the UAV's current local NED position."""
    ## Check this to make blocking optional parameter
    msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.001)
    # This does not work. I'am not sure why
    # msg = conn.recv_match(type="LOCAL_POSITION_NED")
    if msg:
        return global_switch_local_ned((msg.x, msg.y, msg.z))
    return None


TYPE_MASK = int(0b110111111000)


def exec_go_local(
    conn: MAVConnection, _verbose: int, wp: Position, ask_pos_interval: int = 100_000
):
    """Send a MAVLink command to move the UAV to a local waypoint."""
    wp = global_switch_local_ned(wp)
    go_msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,
        conn.target_system,
        conn.target_component,
        Frame.LOCAL_NED,
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
    ask_msg(conn, MsgID.LOCAL_POSITION_NED, interval=ask_pos_interval)


def check_reach_wp(
    conn: MAVConnection,
    verbose: int,
    wp: Position = (0, 0, 10),
    wp_margin: float = 0.5,
):
    """Check if the UAV has reached the target altitude within an acceptable margin."""
    pos = get_local_position(conn)
    if pos is not None:
        dist = math.dist(pos, wp)
        if verbose > 1:
            print(f"Vehicle {conn.target_system}:📍 Distance to target: {dist:.2f} m")
        answer = dist < wp_margin
    else:
        answer = False
    if answer:
        stop_msg(conn, MsgID.LOCAL_POSITION_NED)
    return answer, pos


def fmt(wp: Position):
    """Format a waypoint as a tuple of readable values."""
    return tuple(int(x) if float(x).is_integer() else round(float(x), 2) for x in wp)
