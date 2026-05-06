"""
Module defining the ARM action for vehicle mission planning.

Includes logic to send the ARM command via MAVLink, verify arm status using
HEARTBEAT messages, and construct a corresponding Action object for integration
into mission plans.
"""

import logging
import time

from simulator.helpers.connections.mavlink.enums import Cmd, ModeFlag
from simulator.planner.action import Action
from simulator.planner.step import Step

_RETRY_INTERVAL = 3.0


class Arm(Step):
    """Step to arm the vehicle."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._last_attempt: float = 0.0

    def _send_arm(self) -> None:
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
            0,
        )
        self.mav_manager.send(msg)
        self._last_attempt = time.monotonic()

    def exec_fn(self) -> None:
        """Send ARM command to the vehicle."""
        self._send_arm()

    def check_fn(self) -> bool:
        """Check if the vehicle is armed; retry if rejected."""
        hb = self.mav_manager.state.get("HEARTBEAT")
        if hb and hb.base_mode & ModeFlag.SAFETY_ARMED:
            return True

        if time.monotonic() - self._last_attempt >= _RETRY_INTERVAL:
            ack = self.mav_manager.state.get("COMMAND_ACK")
            if ack is not None and ack.command == Cmd.COMPONENT_ARM_DISARM:
                logging.debug(
                    "Vehicle %s: ARM rejected (result=%s) — retrying...",
                    self.sysid,
                    ack.result,
                )
                self.mav_manager.state.messages.pop("COMMAND_ACK", None)
            else:
                logging.debug("Vehicle %s: ARM not confirmed — retrying...", self.sysid)
            self._send_arm()

        return False


def make_arm() -> Action[Step]:
    """Build an Action to arm the vehicle, including exec and check logic."""
    name = Action.Names.ARM
    arm = Action[Step](name=name, emoji=name.emoji)
    arm.add(Arm("arm"))
    return arm
