"""
Module defining the ARM action for UAV mission planning.

Includes logic to send the ARM command via MAVLink, verify arm status using
HEARTBEAT messages, and construct a corresponding Action object for integration
into mission plans.
"""

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import Cmd, ModeFlag
from plan.core import Action, ActionNames, Step, StepFailed


def make_arm() -> Action[Step]:
    """Build an Action to arm the UAV, including exec and check logic."""
    arm = Action[Step](name=ActionNames.ARM, emoji="🔐")

    class Arm(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """Send ARM command to the UAV."""
            conn.mav.command_long_send(
                conn.target_system,
                conn.target_component,
                Cmd.COMPONENT_ARM_DISARM,
                0,
                1,  # Param 1: 1 = arm, 0 = disarm
                0,
                0,
                0,
                0,
                0,
                0,  # 1 = arm
            )

        def check_fn(self, conn: MAVConnection) -> bool:
            msg = conn.recv_match(type="HEARTBEAT")
            if msg:
                if msg.base_mode & ModeFlag.SAFETY_ARMED:
                    return True
                raise StepFailed(f"flag {msg.base_mode}")
            return False

    arm.add(Arm("arm"))
    return arm
