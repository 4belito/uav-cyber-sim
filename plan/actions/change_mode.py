"""
Defines actions for changing UAV flight modes using MAVLink commands.

Includes logic for creating a mode-switching Action with execution and verification
steps based on HEARTBEAT messages and supported flight modes.
"""

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import CopterMode, ModeFlag
from plan.core import Action, ActionNames, Step


def make_set_mode(flight_mode: CopterMode, onair: bool = False) -> Action[Step]:
    """Create an Action to switch the UAV flight mode."""
    action = Action[Step](
        f"{ActionNames.CHANGE_FLIGHTMODE}: {flight_mode.name}", emoji="⚙️ "
    )

    class SwitchMode(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """Send the SET_MODE command to the UAV with the given mode value."""
            conn.mav.set_mode_send(
                conn.target_system,
                ModeFlag.CUSTOM_MODE_ENABLED,
                flight_mode.value,
            )

        def check_fn(self, conn: MAVConnection) -> bool:
            """Verify the UAV has switched to the target flight mode."""
            msg = conn.recv_match(type="HEARTBEAT")
            if msg and msg.custom_mode == flight_mode.value:
                return True
            return False

    step = SwitchMode(name=f"Switch to {flight_mode.name}")
    action.add(step)
    return action
