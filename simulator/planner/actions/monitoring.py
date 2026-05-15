"""
Mission monitoring helpers for ArduPilot-based vehicles.
"""

import logging
from typing import Literal

from simulator.helpers.connections.mavlink.enums import (
    ModeFlag,
    MsgID,
    PlaneMode,
)
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.planner.action import Action
from simulator.planner.actions.change_mode import SwitchMode
from simulator.planner.plan import CopterMode
from simulator.planner.step import Step


class CheckEndMission(Step):
    """Wait for the vehicle to disarm, confirming it has physically stopped."""

    def __init__(self, name: str):
        super().__init__(name)
        self._was_armed: bool = False

    def exec_fn(self) -> None:
        pass  # HEARTBEAT is always streaming; no setup needed

    def check_fn(self) -> bool:
        hb = self.mav_manager.state.get("HEARTBEAT")
        if hb is None:
            return False
        is_armed = bool(hb.base_mode & ModeFlag.SAFETY_ARMED)
        if is_armed:
            self._was_armed = True
        elif self._was_armed:
            logging.info(
                f"Vehicle {self.sysid}: 🏁 Mission complete - vehicle disarmed"
            )
            return True
        return False


class MonitorItems(Step):
    """Track mission progress via MISSION_CURRENT.seq."""

    def __init__(self, name: str, last_item_seq: int):
        super().__init__(name)
        self._last_seen_seq: int | None = None
        self._last_item_seq = last_item_seq

    def exec_fn(self) -> None:
        msg = ask_msg(conn=self.conn, msg_id=MsgID.MISSION_CURRENT, interval=100_000)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        msg = self.mav_manager.state.get("MISSION_CURRENT")
        if msg is None:
            return False

        current_seq = msg.seq

        if self._last_seen_seq is None:
            self._last_seen_seq = current_seq
            logging.info(f"Vehicle {self.sysid}: ▶️ Current mission item: {current_seq}")
            return False

        if current_seq != self._last_seen_seq:
            completed_seq = self._last_seen_seq

            logging.info(
                f"Vehicle {self.sysid}: ✅ Completed mission item: {completed_seq}"
            )
            logging.info(f"Vehicle {self.sysid}: ▶️ Current mission item: {current_seq}")

            self._last_seen_seq = current_seq

        if current_seq >= self._last_item_seq:
            logging.info(f"Vehicle {self.sysid}: ▶️ Final mission item active")
            self.mav_manager.send(
                stop_msg(conn=self.conn, msg_id=MsgID.MISSION_CURRENT)
            )
            return True

        return False


def make_monitoring(
    item_count: int, firmware: Literal["ArduCopter", "ArduPlane"]
) -> Action[Step]:
    """Monitor mission progress and wait for the vehicle to disarm."""
    name = Action.Names.MONITOR_MISSION
    monitoring = Action[Step](name=name, emoji=name.emoji)
    monitoring.add(MonitorItems(name="monitor items", last_item_seq=item_count))
    monitoring.add(CheckEndMission(name="check end mission"))

    # Switch to STABILIZE/MANUAL to reset the ArduPilot state machine.
    reset_mode: CopterMode | PlaneMode
    match firmware:
        case "ArduCopter":
            reset_mode = CopterMode.STABILIZE
        case "ArduPlane":
            reset_mode = PlaneMode.MANUAL

    monitoring.add(SwitchMode(name="Switch to manual", flight_mode=reset_mode))
    return monitoring
