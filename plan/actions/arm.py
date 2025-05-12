from functools import partial

from pymavlink import mavutil
from pymavlink.dialects.v20.common import MAVLink_heartbeat_message

from plan.core import Action, ActionNames, Step
from plan.mav_helpres import MAVCommand, MAVConnection


def exec_arm(conn: MAVConnection) -> None:
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


def check_arm(conn: MAVConnection, _verbose: int) -> tuple[bool, None]:
    """Check if the UAV is armed using a HEARTBEAT message.

    Returns:
        Tuple[bool, None]: True if armed, False otherwise. Second value reserved for position.
    """
    msg = conn.recv_match(type="HEARTBEAT")
    armed = msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return armed, None
    # if isinstance(msg, MAVLink_heartbeat_message):
    #     armed = (msg.base_mode & MAVCommand.ARMED_FLAG) != 0
    # else:
    #     armed = False  # Could not confirm state

    # return armed, None


def make_arm():
    arm = Action(name=ActionNames.ARM, emoji="🔐")
    arm.add(
        Step(
            "arm",
            check_fn=partial(check_arm),
            exec_fn=partial(exec_arm),
            onair=False,
        )
    )
    return arm
