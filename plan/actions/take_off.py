from pymavlink import mavutil
from functools import partial
import numpy as np

# Custom Modules
from plan.core import Step, Action, ActionNames

# from plan.actions.navegation import check_reach_wp
from plan.actions.navegation import get_local_position

EXT_STATE = mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
REQ_MSG = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
TAKEOFF = mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF


def exec_takeoff(conn: mavutil.mavlink_connection, altitude: float = 1.0):
    """Send a MAVLink command to take off to a specified altitude."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )


def check_takeoff(conn: mavutil.mavlink_connection, _verbose: int):
    # Request EXTENDED_SYS_STATE message
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
        0,
    )
    msg = conn.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=0.01)
    return (msg and msg.landed_state == TAKEOFF), None


def make_takeoff(altitude: float = 1.0) -> Action:
    """
    Creates a takeoff action that consists of a single step:
    - Executing the takeoff command
    - Checking if the UAV reaches the desired altitude
    """
    takeoff_action = Action(name=ActionNames.TAKEOFF, emoji="🛫")
    target_pos = np.array([0, 0, altitude])
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
