
from pymavlink import mavutil
from functools import partial
from plan.core import Step, Action, StepFailed 



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


    @classmethod
    def get_name(cls, value: int) -> str:
        for key, val in cls.__dict__.items():
            if val == value:
                return key
        raise ValueError(f"No mode name for value {value}")

    @classmethod
    def get_value(cls, name: str) -> int:
        try:
            return getattr(cls, name.upper())
        except AttributeError:
            raise ValueError(f"No mode value for name '{name}'")


def exec_set_mode(conn: mavutil.mavlink_connection, blocking: bool = False,mode:int=0) -> None:
    """
    Sends the SET_MODE command to switch to GUIDED mode.
    """
    conn.set_mode(mode)

def check_set_mode(conn: mavutil.mavlink_connection, blocking: bool = False,mode:int=0) -> bool:
    """
    Confirms the UAV has entered GUIDED mode via heartbeat.
    """
    msg = conn.recv_match(type="HEARTBEAT", blocking=blocking, timeout=2)
    if not msg:
        return False

    if msg.custom_mode != mode:
        raise StepFailed(f"Mode not set to {FlightMode.get_name(mode)} (current: {FlightMode.get_name(msg.custom_mode)})")

    return True   

def make_set_mode(mode_name: str) -> Action:
    mode_value = FlightMode.get_value(mode_name)
    action = Action(f"Set Mode: {mode_name.upper()}")
    exec_fn = partial(exec_set_mode, mode=mode_value)
    check_fn = partial(check_set_mode, mode=mode_value)
    step = Step(name=f"Switch to {mode_name.upper()}", check_fn=check_fn, exec_fn=exec_fn)
    action.add(step)
    return action