"""
Module defining the ARM action for UAV mission planning.

Includes logic to send the ARM command via MAVLink, verify arm status using
HEARTBEAT messages, and construct a corresponding Action object for integration
into mission plans.
"""

from functools import partial

from helpers.mavlink import MavCmd, MAVConnection
from plan.core import Action, ActionNames, Step, StepFailed


def make_arm() -> Action[Step]:
    """Build an Action to arm the UAV, including exec and check logic."""
    arm = Action[Step](name=ActionNames.ARM, emoji="🔐")
    arm.add(
        Step(
            "arm",
            check_fn=partial(check_arm),
            exec_fn=partial(exec_arm),
            onair=False,
        )
    )
    return arm


def exec_arm(conn: MAVConnection) -> None:
    """Send ARM command to the UAV."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        MavCmd.ARM_DISARM,
        0,
        1,  # Param 1: 1 = arm, 0 = disarm
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
    if msg:
        if msg.base_mode & MavCmd.ARMED_FLAG:
            return True, None
        raise StepFailed(f"flag {msg.base_mode}")

    # armed = bool(msg and (msg.base_mode & MavCmd.ARMED_FLAG))
    return False, None
