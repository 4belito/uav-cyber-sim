"""Module defining the START_MISSION action for UAV mission planning."""

import logging

from helpers.connections.mavlink.enums import Cmd
from planner.action import Action
from planner.step import Step


class StartMission(Step):
    """Step to start the UAV mission."""

    def exec_fn(self) -> None:
        """Send MISSION_START command to begin executing the mission."""
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
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

    def check_fn(self) -> bool:
        """Check if the mission has started by listening for a STATUSTEXT message."""
        msg = self.conn.recv_match(type="STATUSTEXT")
        if msg:
            text = msg.text.strip().lower()
            if text.startswith("mission"):
                logging.info(
                    f"🚀 Vehicle {self.conn.target_system}: Mission has started"
                )
                return True
        return False


def make_start_mission() -> Action[Step]:
    """Build an Action to start the mission."""
    name = Action.Names.START_MISSION
    arm = Action[Step](name=name, emoji=name.emoji)

    arm.add(StartMission(name="start mission"))
    return arm
