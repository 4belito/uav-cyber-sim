"""
Defines actions for changing UAV flight modes using MAVLink commands.

Includes logic for creating a mode-switching Action with execution and verification
steps based on HEARTBEAT messages and supported flight modes.
"""

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import CopterMode, ModeFlag
from plan.core import Action, ActionNames, Step


class SwitchMode(Step):
    """Step to switch the UAV flight mode."""

    def __init__(self, name: str, flight_mode: CopterMode) -> None:
        super().__init__(name)
        self.flight_mode = flight_mode

    def exec_fn(self, conn: MAVConnection) -> None:
        """Send the SET_MODE command to the UAV with the given mode value."""
        conn.mav.set_mode_send(
            conn.target_system,
            ModeFlag.CUSTOM_MODE_ENABLED,
            self.flight_mode.value,
        )

    def check_fn(self, conn: MAVConnection) -> bool:
        """Verify the UAV has switched to the target flight mode."""
        msg = conn.recv_match(type="HEARTBEAT")
        if msg and msg.custom_mode == self.flight_mode.value:
            return True
        return False


def make_set_mode(flight_mode: CopterMode) -> Action[Step]:
    """Create an Action to switch the UAV flight mode."""
    name = ActionNames.CHANGE_FLIGHTMODE
    action = Action[Step](name, emoji=name.emoji)
    step = SwitchMode(name=f"Switch to {flight_mode.name}", flight_mode=flight_mode)
    action.add(step)
    return action
