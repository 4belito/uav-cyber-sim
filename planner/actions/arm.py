"""
Module defining the ARM action for UAV mission planning.

Includes logic to send the ARM command via MAVLink, verify arm status using
HEARTBEAT messages, and construct a corresponding Action object for integration
into mission plans.
"""

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import Cmd, ModeFlag
from planner.action import Action
from planner.step import Step


class Arm(Step):
    """Step to arm the UAV."""

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
        """Check if the UAV is armed by inspecting HEARTBEAT messages."""
        msg = conn.recv_match(type="HEARTBEAT")
        if msg:
            if msg.base_mode & ModeFlag.SAFETY_ARMED:
                return True
        return False


def make_arm() -> Action[Step]:
    """Build an Action to arm the UAV, including exec and check logic."""
    name = Action.Names.ARM
    arm = Action[Step](name=name, emoji=name.emoji)
    arm.add(Arm("arm"))
    return arm
