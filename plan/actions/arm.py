from pymavlink import mavutil
from plan.core import Step, Action, StepFailed
from functools import partial
from plan.core import ActionNames


class MAVCommand:
    ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
    REQUEST_MESSAGE = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
    LOITER_UNLIMITED = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM


def exec_arm(conn: mavutil.mavlink_connection) -> None:
    """
    Send ARM command to the UAV.
    """
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,  # 1 = arm
    )


def check_arm(conn: mavutil.mavlink_connection) -> bool:
    """Check if the UAV is armed via HEARTBEAT."""
    msg = conn.recv_match(type="HEARTBEAT")
    if not msg:
        return False, None  # Still waiting
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    if not is_armed:
        return False, None
    return True, None


def make_arm():
    arm = Action(ActionNames.ARM)
    arm.add(
        Step(
            "arm",
            check_fn=partial(check_arm),
            exec_fn=partial(exec_arm),
            onair=False,
        )
    )
    return arm
