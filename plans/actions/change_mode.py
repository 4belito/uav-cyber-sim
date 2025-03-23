
from pymavlink import mavutil
from plans.planner import Step, Action, StepFailed 



class FlightMode:
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTO_TUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    AVOID_ADSB = 19
    GUIDED_NO_GPS = 20
    SMART_RTL = 21
    FLOWHOLD = 22
    FOLLOW = 23
    ZIGZAG = 24
    SYSTEM_ID = 25




def exec_guided_mode(conn: mavutil.mavlink_connection, blocking: bool = False) -> None:
    """
    Sends the SET_MODE command to switch to GUIDED mode.
    """
    conn.set_mode(FlightMode.GUIDED)

def check_guided_mode(conn: mavutil.mavlink_connection, blocking: bool = False) -> bool:
    """
    Confirms the UAV has entered GUIDED mode via heartbeat.
    """
    msg = conn.recv_match(type="HEARTBEAT", blocking=blocking, timeout=2)
    if not msg:
        return False

    if msg.custom_mode != FlightMode.GUIDED:
        raise StepFailed(f"Mode not set to GUIDED (current: {msg.custom_mode})")

    return True   




def make_set_guided_mode():
    mode_guided = Action("Guided Mode")   
    mode_guided .add(Step("Change to Guided Mode",check_fn=check_guided_mode,exec_fn=exec_guided_mode))
    return mode_guided