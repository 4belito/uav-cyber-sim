"""
Upload mission action module.

Defines mission monitoring helpers for ArduPilot-based vehicles.
This version is safer for ArduPlane because MISSION_CURRENT is treated as the
currently active item, not as proof that prior items were physically reached.
"""

import logging

from simulator.helpers.connections.mavlink.enums import MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.planner.action import Action
from simulator.planner.step import Step

_STREAM_RETRY_INTERVAL = 0.5  # seconds between retries while waiting for MAVLink 2


class CheckEndMission(Step):
    """Check for mission completion after the last mission item becomes active."""

    def __init__(self, name: str):
        super().__init__(name)

    def exec_fn(self) -> None:
        self.mav_manager.state.messages.pop("MISSION_CURRENT", None)
        self.mav_manager.send(
            ask_msg(conn=self.conn, msg_id=MsgID.MISSION_ITEM_REACHED, interval=100_000)
        )
        self.mav_manager.send(
            ask_msg(self.conn, msg_id=MsgID.MISSION_CURRENT, interval=100_000)
        )

    def check_fn(self) -> bool:
        """Detect mission completion robustly for Plane/Copter."""
        current_msg = self.mav_manager.state.get("MISSION_CURRENT")
        reached_msg = self.mav_manager.state.get("MISSION_ITEM_REACHED")
        if (
            reached_msg
            and current_msg
            and reached_msg.seq >= 1
            and current_msg.seq == 1
        ):
            logging.info(f"Vehicle {self.sysid}: Mission completed")
            return True
        return False


class MonitorItems(Step):
    """Check mission items."""

    def __init__(self, name: str):
        super().__init__(name)
        self._next_seqitem = 1
        self._total = 0

    def exec_fn(self) -> None:
        """Fetch mission size via MISSION_REQUEST_LIST, then start stream."""
        while self._total == 0:
            self.conn.mav.mission_request_list_send(
                self.conn.target_system,
                self.conn.target_component,
            )
            count_msg = self.mav_manager.state.wait_for(
                "MISSION_COUNT", timeout=_STREAM_RETRY_INTERVAL
            )
            if count_msg is not None and count_msg.count > 0:
                self._total = count_msg.count
                logging.debug(f"Vehicle {self.sysid}: mission size = {self._total}")

        self.mav_manager.state.messages.pop("MISSION_CURRENT", None)
        self.mav_manager.send(
            ask_msg(conn=self.conn, msg_id=MsgID.MISSION_CURRENT, interval=100_000)
        )

    def check_fn(self) -> bool:
        msg = self.mav_manager.state.get("MISSION_CURRENT")
        if msg is None:
            return False
        if msg.seq >= self._next_seqitem:
            logging.info(f"Vehicle {self.sysid}: ⭐ Reached item: {msg.seq}")
            self._next_seqitem = msg.seq + 1
        if self._next_seqitem > self._total:
            logging.info(f"Vehicle {self.sysid}: 🏁 Reached all items")
            self.mav_manager.send(
                stop_msg(conn=self.conn, msg_id=MsgID.MISSION_CURRENT)
            )
            return True
        self.mav_manager.state.messages.pop("MISSION_CURRENT", None)
        return False


def make_monitoring() -> Action[Step]:
    """Monitor mission progress and completion."""
    name = Action.Names.MONITOR_MISSION
    monitoring = Action[Step](name=name, emoji=name.emoji)
    monitoring.add(MonitorItems(name="monitor items"))
    monitoring.add(CheckEndMission(name="check end mission"))
    return monitoring
