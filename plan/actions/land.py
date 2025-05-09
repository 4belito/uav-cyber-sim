from functools import partial

import numpy as np
from pymavlink import mavutil

from plan.actions.navegation import get_local_position

# Custom Modules
from plan.core import Action, ActionNames, Step

LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
REQ_MSG = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
EXT_STATE = mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND


def check_land(conn: mavutil.mavlink_connection, verbose: int):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        REQ_MSG,
        0,
        EXT_STATE,
        0,
        0,
        0,
        0,
        0,
        0,
    )  # parameter 4 is confirmation(it may be increased)
    msg = conn.recv_match(type="EXTENDED_SYS_STATE")
    current_pos = get_local_position(conn)
    if current_pos is not None and verbose > 1:
        print(f"Vehicle {conn.target_system}: 🛬 Altitute: {current_pos[2]:.2f} m")
    return (msg and msg.landed_state == ON_GROUND), current_pos


def exec_land(conn: mavutil.mavlink_connection):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component, LAND, 0, 0, 0, 0, 0, 0, 0, 0
    )


def make_land(final_wp: np.ndarray):
    example_action = Action(name=ActionNames.LAND, emoji="🛬")
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
