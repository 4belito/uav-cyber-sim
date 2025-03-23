from pymavlink import mavutil
from functools import partial
from plans.planner import Step, Action, StepFailed 

class ParamType:
    INT8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
    INT16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16
    INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32


class ParamName:
    """
    MAVLink parameter names used to configure UAV behavior.

    Each parameter is represented as a `bytes` object (max 16 characters),
    as required by the MAVLink protocol.
    """

    NAV_SPEED = b'WPNAV_SPEED'  # Horizontal speed (cm/s) for waypoint navigation

    @classmethod
    def get_name(cls, value: bytes) -> str:
        for key, val in cls.__dict__.items():
            if isinstance(val, bytes) and val == value:
                return key
        raise ValueError(f"No parameter name for value {value}")

    @classmethod
    def get_value(cls, name: str) -> bytes:
        try:
            return getattr(cls, name.upper())
        except AttributeError:
            raise ValueError(f"No parameter value for name '{name}'")


def exec_set_nav_speed(conn: mavutil.mavlink_connection, blocking: bool = False, speed: float = 5) -> None:
    """
    Sends a SET_PARAM command to change WPNAV_SPEED (navigation speed).
    """
    speed_cmps = speed * 100  # ArduPilot uses cm/s
    print(f"📤 Sending WPNAV_SPEED = {speed_cmps:.0f} cm/s ({speed:.2f} m/s)")
    conn.mav.param_set_send(
        conn.target_system,
        conn.target_component,
        ParamName.NAV_SPEED,
        speed_cmps,
        ParamType.REAL32
    )


def check_set_nav_speed(conn: mavutil.mavlink_connection, blocking: bool = False, speed: float = 0) -> bool:
    """
    Checks whether the WPNAV_SPEED parameter has been updated.
    """
    msg = conn.recv_match(type='PARAM_VALUE', blocking=blocking, timeout=2)
    if not msg:
        return False

    # Clean param_id bytes
    expected = ParamName.NAV_SPEED.decode('utf-8')
    speed_cmps = speed * 100

    if msg.param_id == expected and msg.param_value == speed_cmps:
        print(f"✅ Parameter '{msg.param_id}' confirmed at {msg.param_value:.0f} cm/s ({speed:.2f} m/s)")
        return True
    else:
        raise StepFailed(f"Parameter '{msg.param_id}' value {msg.param_value:.1f} ≠ {speed_cmps:.1f}")


def make_change_nav_speed(speed: float) -> Action:
    """
    Returns an Action that changes the UAV's WPNAV_SPEED.
    """
    action = Action("Set Navigation Speed")
    exec_fn = partial(exec_set_nav_speed, speed=speed)
    check_fn = partial(check_set_nav_speed, speed=speed)
    step = Step(name=f"Set speed to {speed:.2f} m/s", check_fn=check_fn, exec_fn=exec_fn)
    action.add(step)
    return action
