"""Module defining the START_MISSION action for UAV mission planning."""

import logging

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import Cmd
from plan.core import Action, ActionNames, Step


def make_start_mission() -> Action[Step]:
    """Build an Action to start the mission."""
    arm = Action[Step](name=ActionNames.START_MISSION, emoji="🚀")

    class StartMission(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """Send MISSION_START command to begin executing the mission."""
            exec_start_mission(conn)

        def check_fn(self, conn: MAVConnection) -> bool:
            """Check if the mission has started."""
            return check_start_mission(conn)

    arm.add(StartMission(name="start mission"))
    return arm


def exec_start_mission(conn: MAVConnection) -> None:
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


def check_start_mission(conn: MAVConnection) -> bool:
    """Check if the mission has started by listening for a STATUSTEXT message."""
    msg = conn.recv_match(type="STATUSTEXT")
    if msg:
        text = msg.text.strip().lower()
        if text.startswith("mission"):
            logging.info(f"🚀 Vehicle {conn.target_system}: Mission has started")
            return True
    return False
