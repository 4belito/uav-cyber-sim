"""
Module defining the ARM action for vehicle mission planning.

Includes logic to send the ARM command via MAVLink, verify arm status using
HEARTBEAT messages, and construct a corresponding Action object for integration
into mission plans.
"""

from simulator.helpers.connections.mavlink.enums import Cmd, ModeFlag
from simulator.planner.action import Action
from simulator.planner.step import Step


class Arm(Step):
    """Step to arm the vehicle."""

    def exec_fn(self) -> None:
        """Send ARM command to the vehicle."""
        msg = self.conn.mav.command_long_encode(
            self.conn.target_system,
            self.conn.target_component,
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
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Check if the vehicle is armed by inspecting HEARTBEAT messages."""
        msg = self.mav_manager.state.get("HEARTBEAT")
        if msg:
            if msg.base_mode & ModeFlag.SAFETY_ARMED:
                return True
        return False


def make_arm() -> Action[Step]:
    """Build an Action to arm the vehicle, including exec and check logic."""
    name = Action.Names.ARM
    arm = Action[Step](name=name, emoji=name.emoji)
    arm.add(Arm("arm"))
    return arm
