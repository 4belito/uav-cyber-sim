
from pymavlink import mavutil
from plans.planner import Step, Action



LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
REQ_MSG = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
EXT_STATE = mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND

def check_land(conn: mavutil.mavlink_connection, blocking=False):
    conn.mav.command_long_send(conn.target_system, conn.target_component,REQ_MSG,0,EXT_STATE,0, 0, 0, 0, 0, 0 ) #parameter 4 is confirmation(it may be increased)
    msg = conn.recv_match(type='EXTENDED_SYS_STATE', blocking=blocking)
    if msg:
        return msg.landed_state == ON_GROUND
    else:
        return False

def exec_land(conn: mavutil.mavlink_connection, blocking=False):
    conn.mav.command_long_send(conn.target_system, conn.target_component, LAND , 0, 0, 0, 0, 0, 0, 0, 0)



def make_land():
    example_action = Action("Land")
    example_action.add(Step("land",check_fn=check_land,exec_fn=exec_land))
    return example_action


