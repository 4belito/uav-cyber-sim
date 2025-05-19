"""
Defines actions for changing UAV flight modes using MAVLink commands.

Includes logic for creating a mode-switching Action with execution and verification
steps based on HEARTBEAT messages and supported flight modes.
"""

from functools import partial

from helpers.mavlink import FlightMode, MAVConnection
from plan.core import Action, ActionNames, Step


def make_set_mode(mode_name: str, onair: bool = False) -> Action:
    """Creates an Action to switch the UAV flight mode."""
    mode_value = FlightMode.get_value(mode_name)
    action = Action(f"{ActionNames.CHANGE_FLIGHTMODE}: {mode_name.upper()}", emoji="⚙️")
    exec_fn = partial(exec_set_mode, mode=mode_value)
    check_fn = partial(check_set_mode, mode=mode_value)
    step = Step(
        name=f"Switch to {mode_name.upper()}",
        check_fn=check_fn,
        exec_fn=exec_fn,
        onair=onair,
    )
    action.add_step(step)
    return action


def exec_set_mode(conn: MAVConnection, mode: int = 0) -> None:
    """Sends the SET_MODE command to the UAV with the given mode value."""
    conn.set_mode(mode)


def check_set_mode(
    conn: MAVConnection, _verbose: int, mode: int = 0
) -> tuple[bool, None]:
    """Verifies the UAV has switched to the target flight mode."""
    msg = conn.recv_match(type="HEARTBEAT", timeout=1.0)
    if msg and msg.custom_mode == mode:
        return True, None
    return False, None
