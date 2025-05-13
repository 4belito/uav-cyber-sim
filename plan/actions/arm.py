"""
Module defining the ARM action for UAV mission planning.

Includes logic to send the ARM command via MAVLink, verify arm status using
HEARTBEAT messages, and construct a corresponding Action object for integration
into mission plans.
"""

from functools import partial

from plan.core import Action, ActionNames, Step
from helpers.mavlink import MAVCommand, MAVConnection


def make_arm():
    """Builds an Action to arm the UAV, including exec and check logic."""
    arm = Action(name=ActionNames.ARM, emoji="🔐")
    arm.add_step(
        Step(
            "arm",
            check_fn=partial(check_arm),
            exec_fn=partial(exec_arm),
            onair=False,
        )
    )
    return arm


def exec_arm(conn: MAVConnection) -> None:
    """
    Send ARM command to the UAV.
    """
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        MAVCommand.ARM_DISARM,
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
    """Check if the UAV is armed using a HEARTBEAT message."""
    msg = conn.recv_match(type="HEARTBEAT")
    armed = bool(msg and (msg.base_mode & MAVCommand.ARMED_FLAG))
    return armed, None
