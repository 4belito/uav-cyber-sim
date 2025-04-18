from pymavlink import mavutil
from functools import partial
from plan.core import Step, Action, StepFailed, ActionNames


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


def exec_set_mode(conn: mavutil.mavlink_connection, mode: int = 0) -> None:
    """
    Sends the SET_MODE command to switch to GUIDED mode.
    """
    conn.set_mode(mode)


def check_set_mode(
    conn: mavutil.mavlink_connection, _verbose: int, mode: int = 0
) -> bool:
    """
    Confirms the UAV has entered GUIDED mode via heartbeat.
    """
    msg = conn.recv_match(type="HEARTBEAT")
    return (msg and msg.custom_mode == mode), None


def make_set_mode(mode_name: str, onair=False) -> Action:
    mode_value = FlightMode.get_value(mode_name)
    action = Action(
        f"{ActionNames.CHANGE_FLIGHTMODE}: {mode_name.upper()}", emoji="⚙️"
    )  # This may be improved in the future
    exec_fn = partial(exec_set_mode, mode=mode_value)
    check_fn = partial(check_set_mode, mode=mode_value)
    step = Step(
        name=f"Switch to {mode_name.upper()}",
        check_fn=check_fn,
        exec_fn=exec_fn,
        onair=onair,
    )
    action.add(step)
    return action
