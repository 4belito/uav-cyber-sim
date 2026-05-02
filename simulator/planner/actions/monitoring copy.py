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

_RETRY_TICKS = 50  # re-request stream every ~0.5 s when no messages arrive


class CheckEndMission(Step):
    """Check for mission completion after the last mission item becomes active."""

    def __init__(self, name: str):
        super().__init__(name)
        self._idle_ticks = 0

    def exec_fn(self) -> None:
        self._request_streams()

    def _request_streams(self) -> None:
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
        if current_msg is None or reached_msg is None:
            self._idle_ticks += 1
            if self._idle_ticks % _RETRY_TICKS == 0:
                self._request_streams()
            return False
        self._idle_ticks = 0
        if reached_msg.seq >= 1 and current_msg.seq == 1:
            logging.info(f"Vehicle {self.sysid}: Mission completed")
            return True
        return False


class MonitorItems(Step):
    """
    Check mission items
    """

    def __init__(self, name: str):
        super().__init__(name)
        self._next_seqitem = 1
        self._idle_ticks = 0

    def exec_fn(self) -> None:
        self._request_stream()

    def _request_stream(self) -> None:
        self.mav_manager.send(
            ask_msg(conn=self.conn, msg_id=MsgID.MISSION_CURRENT, interval=100_000)
        )

    def check_fn(self) -> bool:
        msg = self.mav_manager.state.get("MISSION_CURRENT")
        if msg is None:
            self._idle_ticks += 1
            if self._idle_ticks % _RETRY_TICKS == 0:
                self._request_stream()
            return False
        self._idle_ticks = 0
        total: int = getattr(msg, "total", 0)
        if msg.seq >= self._next_seqitem:
            logging.info(f"Vehicle {self.sysid}: ⭐ Reached item: {msg.seq}")
            self._next_seqitem = msg.seq + 1
        if total > 0 and self._next_seqitem > total:
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
