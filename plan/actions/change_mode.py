"""
Defines actions for changing UAV flight modes using MAVLink commands.

Includes logic for creating a mode-switching Action with execution and verification
steps based on HEARTBEAT messages and supported flight modes.
"""

from functools import partial

from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import CmdDo, CopterMode, ModeFlag
from plan.core import Action, ActionNames, Step


def make_set_mode(flight_mode: CopterMode, onair: bool = False) -> Action[Step]:
    """Create an Action to switch the UAV flight mode."""
    action = Action[Step](
        f"{ActionNames.CHANGE_FLIGHTMODE}: {flight_mode.name}", emoji="⚙️"
    )
    exec_fn = partial(exec_set_mode, mode=flight_mode)
    check_fn = partial(check_set_mode, mode=flight_mode)
    step = Step(
        name=f"Switch to {flight_mode.name}",
        check_fn=check_fn,
        exec_fn=exec_fn,
        onair=onair,
    )
    action.add(step)
    return action


def exec_set_mode(conn: MAVConnection, _verbose: int, mode: CopterMode) -> None:
    """Send the SET_MODE command to the UAV with the given mode value."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        CmdDo.SET_MODE,
        0,
        ModeFlag.CUSTOM_MODE_ENABLED,
        mode.value,
        0,
        0,
        0,
        0,
        0,
    )

    # conn.set_mode(mode)


def check_set_mode(
    conn: MAVConnection, _verbose: int, mode: CopterMode
) -> tuple[bool, None]:
    """Verify the UAV has switched to the target flight mode."""
    msg = conn.recv_match(type="HEARTBEAT", timeout=1.0)
    if msg and msg.custom_mode == mode.value:
        return True, None
    return False, None
