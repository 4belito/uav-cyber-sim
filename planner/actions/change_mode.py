"""
Defines actions for changing UAV flight modes using MAVLink commands.

Includes logic for creating a mode-switching Action with execution and verification
steps based on HEARTBEAT messages and supported flight modes.
"""

from helpers.connections.mavlink.enums import CopterMode, ModeFlag
from planner.action import Action
from planner.step import Step


class SwitchMode(Step):
    """Step to switch the UAV flight mode."""

    def __init__(self, name: str, flight_mode: CopterMode) -> None:
        super().__init__(name)
        self.flight_mode = flight_mode

    def exec_fn(self) -> None:
        """Send the SET_MODE command to the UAV with the given mode value."""
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            ModeFlag.CUSTOM_MODE_ENABLED,
            self.flight_mode.value,
        )

    def check_fn(self) -> bool:
        """Verify the UAV has switched to the target flight mode."""
        msg = self.conn.recv_match(type="HEARTBEAT")
        if msg and msg.custom_mode == self.flight_mode.value:
            return True
        return False


def make_set_mode(flight_mode: CopterMode) -> Action[Step]:
    """Create an Action to switch the UAV flight mode."""
    name = Action.Names.CHANGE_FLIGHTMODE
    action = Action[Step](name, emoji=name.emoji)
    step = SwitchMode(name=f"Switch to {flight_mode.name}", flight_mode=flight_mode)
    action.add(step)
    return action
