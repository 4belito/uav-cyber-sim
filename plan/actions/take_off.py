from pymavlink import mavutil
from functools import partial
import numpy as np

# Custom Modules
from plan.core import Step, Action, ActionNames
from plan.actions.navegation import check_reach_wp


# 🚁 Send takeoff command
def exec_takeoff(conn: mavutil.mavlink_connection, altitude: float = 10.0):
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


def make_takeoff(altitude: float = 10.0, wp_margin=0.5, verbose: int = 0) -> Action:
    """
    Creates a takeoff action that consists of a single step:
    - Executing the takeoff command
    - Checking if the UAV reaches the desired altitude
    """
    takeoff_action = Action(ActionNames.TAKEOFF)
    target_pos = np.array([0, 0, altitude])
    check_fn = partial(check_reach_wp, wp=target_pos, wp_margin=wp_margin)
    exec_fn = partial(exec_takeoff, altitude=altitude)
    step = Step(
        "takeoff", check_fn=check_fn, exec_fn=exec_fn,  onair=True, target_pos=target_pos
    )
    takeoff_action.add(step)
    return takeoff_action
