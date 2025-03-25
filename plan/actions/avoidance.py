
from pymavlink import mavutil
from plan.core import Step, Action
from functools import partial

TYPE_MASK= int(0b010111000111) 
BODY_COORD= mavutil.mavlink.MAV_FRAME_BODY_NED



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


