"""
Defines a TAKEOFF action using MAVLink commands.

Includes:
- exec_takeoff: sends a takeoff command to the UAV.
- check_takeoff: verifies if the UAV is currently taking off.
- make_takeoff: creates a takeoff Action with one execution step.
"""

from functools import partial

from helpers.connections.mavlink.customtypes.location import ENU
from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import CmdNav, LandState, MsgID
from helpers.connections.mavlink.streams import ask_msg, stop_msg
from plan.core import Action, ActionNames, Step


def make_takeoff(altitude: float = 1.0) -> Action[Step]:
    """Create a TAKEOFF action with execution and check steps."""
    takeoff_action = Action[Step](name=ActionNames.TAKEOFF, emoji="🛫")

    target_pos = ENU(0, 0, altitude)
    # check_fn = partial(check_takeoff, wp=target_pos, wp_margin=wp_margin)
    exec_fn = partial(exec_takeoff, altitude=altitude)
    step = Step(
        "takeoff",
        check_fn=check_takeoff,
        exec_fn=exec_fn,
        onair=True,
        target_pos=target_pos,
    )
    takeoff_action.add(step)
    return takeoff_action


def exec_takeoff(conn: MAVConnection, altitude: float = 1.0):
    """Send TAKEOFF command to reach target altitude."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        CmdNav.TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )
    ask_msg(conn, MsgID.EXTENDED_SYS_STATE)


def check_takeoff(conn: MAVConnection):
    """Check if UAV is in TAKEOFF state."""
    msg = conn.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=0.01)
    take_off = bool(msg and msg.landed_state == LandState.TAKEOFF)
    if take_off:
        stop_msg(conn, MsgID.EXTENDED_SYS_STATE)
    return take_off, None
