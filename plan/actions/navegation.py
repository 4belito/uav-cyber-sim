
from pymavlink import mavutil
import numpy as np
from functools import partial

from plan.core import Step, Action
from helpers.change_coordinates import GLOBAL_switch_LOCAL_NED

TYPE_MASK= int(0b110111111000)
LOCAL_COORD= mavutil.mavlink.MAV_FRAME_LOCAL_NED


def get_local_position(conn: mavutil.mavlink_connection,blocking=False):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        0, 0, 0, 0, 0, 0
    )
    msg = conn.recv_match(type='LOCAL_POSITION_NED',blocking=True,timeout=2)
    if msg:
        return np.array(GLOBAL_switch_LOCAL_NED(msg.x, msg.y, msg.z))
    else:
        return False



def exec_go_local(conn: mavutil.mavlink_connection, wp: np.ndarray = np.array([1, 1, 10]),verbose:int=0):
    if verbose: print(f'message to go to {tuple(wp)} sent')
    wp=GLOBAL_switch_LOCAL_NED(*wp)
    go_msg=mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, conn.target_system, conn.target_component, LOCAL_COORD, TYPE_MASK, *wp, 0, 0, 0, 0, 0, 0, 0, 0)
    conn.mav.send(go_msg)
    


# 📡 Check if UAV reached the wp
def check_reach_wp(conn: mavutil.mavlink_connection, wp: np.ndarray = np.array([0, 0, 10]),wp_margin=0.5,verbose: int = 0):
    """Check if the UAV has reached the target altitude within an acceptable margin."""
    pos = get_local_position(conn) 
    
    if pos is False:
        return False
    else:
        # Compute distance to target waypoint
        dist = np.linalg.norm(pos - wp)
        #if verbose:
        print(f"Vehicle {conn.target_system}:📍 Distance to target: {dist:.2f} m")

        # Check if the UAV has reached the waypoint within the margin
        return dist < wp_margin


def make_path(wps:np.ndarray = np.empty((0, 3)),wp_margin:float=0.5,verbose:int=0):
    go_local_action = Action("fly")
    if wps is None or len(wps) == 0:
        return go_local_action  # Return empty action if no waypoints
    for wp in wps:
        go_local_action.add(make_go_to(wp,wp_margin,verbose=verbose))

    return go_local_action

def make_go_to(wp:np.ndarray = np.empty((0, 3)),wp_margin:float=0.5,verbose:int=0,cause_text=''):
    goto_step= Step(f"go to {cause_text} -> {tuple(wp)}",
            check_fn=partial(check_reach_wp,wp=wp,wp_margin=wp_margin,verbose=verbose),
            exec_fn=partial(exec_go_local,wp=wp))
    goto_step.wp=wp
    return goto_step