"""Module defining the START_MISSION action for UAV mission planning."""

from functools import partial

from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import Cmd
from plan.core import Action, ActionNames, Step


def make_start_mission() -> Action[Step]:
    """Build an Action to start the mission."""
    arm = Action[Step](name=ActionNames.START_MISSION, emoji="🚀")
    arm.add(
        Step(
            "start_mission",
            check_fn=partial(check_start_mission),
            exec_fn=partial(exec_start_mission),
            onair=False,
        )
    )
    return arm


def exec_start_mission(conn: MAVConnection, _verbose: int) -> None:
    """Send MISSION_START command to begin executing the mission."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        Cmd.MISSION_START,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def check_start_mission(conn: MAVConnection, verbose: int) -> tuple[bool, None]:
    """Check if the mission has started by listening for a STATUSTEXT message."""
    msg = conn.recv_match(type="STATUSTEXT")
    if msg:
        text = msg.text.strip().lower()
        if text.startswith("mission"):
            if verbose == 2:
                print("🚀 Mission has started")
            return True, None
    return False, None
