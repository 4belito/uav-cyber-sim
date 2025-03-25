
from pymavlink import mavutil
from plan.core import Step, Action
from functools import partial
from helpers.change_coordinates import GLOBAL_switch_LOCAL_NED
TYPE_MASK= int(0b010111000111) 
BODY_COORD= mavutil.mavlink.MAV_FRAME_BODY_NED



from plan.actions.navegation import exec_go_local


import numpy as np
from pymavlink import mavutil

BODY_COORD = mavutil.mavlink.MAV_FRAME_BODY_NED
LOCAL_COORD=mavutil.mavlink.MAV_FRAME_LOCAL_NED
TYPE_MASK_VELOCITY_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

def exec_avoid_move(conn: mavutil.mavlink_connection,
                    pos: np.ndarray,
                    obj_pos:np.ndarray,
                    speed: float = 2.0,
                    direction: str = 'left'):
    """
    Sends a velocity command in body frame, orthogonal to the direction of wp.
    `direction` can be 'left' or 'right' (relative to wp direction).
    """

    # Normalize wp direction (ignore Z)
    dir_vector = (obj_pos -pos)[:2]
    print(dir_vector)
    if np.linalg.norm(dir_vector) == 0:
        print("Warning: Zero waypoint vector.")
        return
    dir_vector = dir_vector / np.linalg.norm(dir_vector)

    # Get orthogonal direction
    if direction == 'left':
        ortho = np.array([-dir_vector[1], dir_vector[0],0])
    elif direction == 'right':
        ortho = np.array([dir_vector[1], -dir_vector[0],0])
    else:
        raise ValueError("Direction must be 'left' or 'right'")

    # Scale to desired speed
    pos = pos+ortho
    exec_go_local(conn, wp=pos)

    print(f"Sent velocity move {direction} at {speed} m/s")

# from plan.actions.navegation import get_local_position

# import math

# def set_avoidance_destination(conn, avoidance_vector, current_heading_deg, max_magnitude=3.0, target_alt=2.0):
#     """
#     Rotate and limit an avoidance vector, then send a position setpoint to avoid.
#     """

#     # Convert heading to radians
#     heading_rad = math.radians(current_heading_deg)

#     # Copy vector so we don't overwrite
#     vx, vy = avoidance_vector

#     # Rotate vector by heading (drone-relative to world)
#     rotated_x = vx * math.cos(heading_rad) - vy * math.sin(heading_rad)
#     rotated_y = vx * math.sin(heading_rad) + vy * math.cos(heading_rad)

#     # Limit vector magnitude
#     norm = math.sqrt(rotated_x**2 + rotated_y**2)
#     if norm > max_magnitude:
#         rotated_x = max_magnitude * (rotated_x / norm)
#         rotated_y = max_magnitude * (rotated_y / norm)

#     # Get current position (you’ll need to get this from LOCAL_POSITION_NED or however you're tracking it)
#     current_pos = get_local_position(conn)  # returns [x, y, z]

#     # Compute destination
#     dest_x = current_pos[0] + rotated_x
#     dest_y = current_pos[1] + rotated_y
#     dest_z = current_pos[2] #target_alt  # fixed height, could also do current_pos[2] if you want to keep height

#     # Send the new position target
#     wp=GLOBAL_switch_LOCAL_NED(dest_x,dest_y,dest_z)
#     msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
#         10,
#         conn.target_system,
#         conn.target_component,
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#         # Use position only
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
#         mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
#         *wp,  # NED: z is negative
#         0, 0, 0,  # velocities ignored
#         0, 0, 0,  # accelerations ignored
#         0, 0      # yaw
#     )
#     conn.mav.send(msg)
#     print(f"Sent avoidance destination to ({dest_x:.2f}, {dest_y:.2f}, {dest_z:.2f})")






def stop_position_hold(conn: mavutil.mavlink_connection):
    msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, conn.target_system, conn.target_component, BODY_COORD,
        # Mask position, acceleration, and yaw, but allow velocity
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
        0, 0, 0,  # position (ignored)
        0, 0, 0,  # velocity (will set next)
        0, 0, 0,  # acceleration (ignored)
        0, 0      # yaw (ignored)
    )
    conn.mav.send(msg)



def exec_move(conn: mavutil.mavlink_connection,speed:float=5.0,direction='left'):
    if direction == 'left':
        vy = -speed
    elif direction == 'right':
         vy = speed
    msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, conn.target_system, conn.target_component, BODY_COORD, TYPE_MASK,
            0, 0, 0,  # Position ignored
            0, vy, 0,  # Move left (velocity command)
            0, 0, 0,  # Acceleration ignored
            0, 0  # Ignore yaw
        )
    conn.mav.send(msg)

def check_move(conn: mavutil.mavlink_connection):
    return True




def make_move(direction:str,speed:float=5.0,verbose:int=0):
    example_action = Action("Example_Action")
    example_action.add(Step("step_1",check_fn=partial(check_move),exec_fn=partial(exec_move,direction=direction,speed=speed)))
    return example_action


