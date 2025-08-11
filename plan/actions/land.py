"""Defines a LAND action with execution and landing check using MAVLink commands."""

import logging

from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import ENU
from mavlink.enums import CmdNav, LandState, MsgID
from mavlink.util import ask_msg, get_ENU_position, stop_msg
from plan.core import Action, ActionNames, Step


def make_land(final_wp: ENU) -> Action[Step]:
    """Create a LAND Action with execution and check steps."""
    example_action = Action[Step](name=ActionNames.LAND, emoji="🛬")
    example_action.add(
        Step(
            "land",
            check_fn=check_land,
            exec_fn=exec_land,
            target_pos=final_wp,
            onair=True,
        )
    )
    return example_action


def exec_land(
    conn: MAVConnection,
    verbose: int,
    ask_pos_interval: int = 100_000,
    ask_land_interval: int = 100_000,
):
    """Send a MAVLink command to initiate landing."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        CmdNav.LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    ask_msg(conn, verbose, MsgID.EXTENDED_SYS_STATE, interval=ask_land_interval)
    ask_msg(conn, verbose, MsgID.LOCAL_POSITION_NED, interval=ask_pos_interval)


def check_land(conn: MAVConnection, verbose: int):
    """Check if the UAV has landed using EXTENDED_SYS_STATE."""
    # parameter 4 is confirmation(it may be increased)
    msg = conn.recv_match(type="EXTENDED_SYS_STATE")
    current_pos = get_ENU_position(conn)
    if current_pos is not None:
        logging.debug(f"Vehicle {conn.target_system}: Altitude: {current_pos[2]:.2f} m")
    on_ground = bool(msg and msg.landed_state == LandState.ON_GROUND)
    if on_ground:
        stop_msg(conn, MsgID.EXTENDED_SYS_STATE)
    return on_ground, current_pos
