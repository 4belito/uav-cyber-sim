from pymavlink import mavutil
from plan.core import Step, Action, ActionNames
from plan.actions.navegation import get_local_position

LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
REQ_MSG = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
EXT_STATE = mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND


def check_land(conn: mavutil.mavlink_connection):
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
    if msg:
        answer = msg.landed_state == ON_GROUND
    else:
        answer = False
    return answer, current_pos


def exec_land(conn: mavutil.mavlink_connection):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component, LAND, 0, 0, 0, 0, 0, 0, 0, 0
    )


def make_land(wp, verbose: int = 0):
    example_action = Action(ActionNames.LAND)
    example_action.add(
        Step("land", check_fn=check_land, exec_fn=exec_land, target_pos=wp)
    )
    return example_action
