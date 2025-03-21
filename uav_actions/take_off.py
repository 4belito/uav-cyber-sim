
from pymavlink import mavutil
from functools import partial
import numpy as np

# Custom Modules
from mission_flow import Step, Action, StepFailed 
from helpers.change_coordinates import GLOBAL_switch_LOCAL_NED



# 🚁 Send takeoff command
def exec_takeoff(conn: mavutil.mavlink_connection, blocking=False, altitude: float = 10.0):
    """Send a MAVLink command to take off to a specified altitude."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )


# 📡 Check if UAV reached the target altitude
def reach_wp(conn: mavutil.mavlink_connection, blocking=False, wp: np.ndarray = np.array([0, 0, 10]),wp_margin=0.5):
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




def make_takeoff_action(altitude: float = 10.0,wp_margin= 0.5) -> Action:
    """
    Creates a takeoff action that consists of a single step:
    - Executing the takeoff command
    - Checking if the UAV reaches the desired altitude
    """
    # Define the step with dynamic altitude
    takeoff_step = Step(
        "take off",
        check_fn=partial(reach_wp, wp=np.array([0, 0, altitude]),wp_margin= wp_margin),
        exec_fn=partial(exec_takeoff, altitude=altitude)
    )

    # Define the action and add the step
    takeoff_action = Action("Take Off")
    takeoff_action.add_step(takeoff_step)

    return takeoff_action