
from pymavlink import mavutil
from functools import partial
import numpy as np

# Custom Modules
from plan.core import Step, Action
from plan.actions.navegation import check_reach_wp




# 🚁 Send takeoff command
def exec_takeoff(conn: mavutil.mavlink_connection, blocking=False, altitude: float = 10.0):
    """Send a MAVLink command to take off to a specified altitude."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )



def make_takeoff(altitude: float = 10.0,wp_margin= 0.5) -> Action:
    """
    Creates a takeoff action that consists of a single step:
    - Executing the takeoff command
    - Checking if the UAV reaches the desired altitude
    """
    takeoff_action = Action("Take Off")    
    takeoff_action.add(Step("take off",
                            check_fn=partial(check_reach_wp, wp=np.array([0, 0, altitude]),wp_margin= wp_margin),
                            exec_fn=partial(exec_takeoff, altitude=altitude)
                            ))
    return takeoff_action