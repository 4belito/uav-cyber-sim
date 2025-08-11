"""
Defines logic for navigating to local NED waypoints using MAVLink.

Includes:
- Functions to command and check UAV movement in local coordinates.
- Construction of Step and Action objects to integrate into mission plans.
"""

import logging
import math
from functools import partial

from pymavlink import mavutil

from helpers.change_coordinates import ENU_to_NED
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import ENU, ENUs
from mavlink.enums import Frame, MsgID
from mavlink.util import ask_msg, get_ENU_position, stop_msg
from plan.core import Action, ActionNames, Step


def make_go_to(
    wp: ENU,
    wp_margin: float = 0.5,
    cause_text: str = "",
    target_pos: ENU | None = None,
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
    )
    return goto_step


## Make the action
def make_path(wps: ENUs | None = None, wp_margin: float = 0.5) -> Action[Step]:
    """Create a FLY action composed of multiple go-to waypoint steps."""
    go_local_action = Action[Step](name=ActionNames.FLY, emoji="🛩️")
    if wps is None:
        return go_local_action  # Return empty action if no waypoints
    for wp in wps:
        go_local_action.add(make_go_to(wp, wp_margin))

    return go_local_action


TYPE_MASK = int(0b110111111000)


def exec_go_local(
    conn: MAVConnection, verbose: int, wp: ENU, ask_pos_interval: int = 100_000
):
    """Send a MAVLink command to move the UAV to a local waypoint."""
    ned_wp = ENU_to_NED(wp)
    go_msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,
        conn.target_system,
        conn.target_component,
        Frame.LOCAL_NED,
        TYPE_MASK,
        *ned_wp,
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
    ask_msg(conn, verbose, MsgID.LOCAL_POSITION_NED, interval=ask_pos_interval)


def check_reach_wp(
    conn: MAVConnection,
    verbose: int,
    wp: ENU = ENU(0, 0, 10),
    wp_margin: float = 0.5,
):
    """Check if the UAV has reached the target altitude within an acceptable margin."""
    pos = get_ENU_position(conn)
    if pos is not None:
        dist = math.dist(pos, wp)
        logging.debug(f"📍 Vehicle {conn.target_system}: Distance to target: {dist:.2f} m")
        answer = dist < wp_margin
    else:
        answer = False
    if answer:
        stop_msg(conn, MsgID.LOCAL_POSITION_NED)
    return answer, pos


def fmt(wp: ENU):
    """Format a waypoint as a tuple of readable values."""
    return tuple(int(x) if float(x).is_integer() else round(float(x), 2) for x in wp)
