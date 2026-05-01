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


class CheckEndMission(Step):
    """Check for mission completion after the last mission item becomes active."""

    def __init__(self, name: str, item_count: int):
        super().__init__(name)
        self._last_item = item_count - 1

    def exec_fn(self) -> None:
        """No execution needed; just checking."""
        reached_msg = ask_msg(
            conn=self.conn, msg_id=MsgID.MISSION_ITEM_REACHED, interval=100_000
        )
        status_msg = ask_msg(self.conn, msg_id=MsgID.STATUSTEXT, interval=100_000)

        self.mav_manager.send(reached_msg)
        self.mav_manager.send(status_msg)

    def check_fn(self) -> bool:
        """Detect mission completion robustly for Plane/Copter."""
        status_msg = self.mav_manager.state.get("STATUSTEXT")
        reached_msg = self.mav_manager.state.get("MISSION_ITEM_REACHED")
        if (
            reached_msg
            and status_msg
            and reached_msg.seq >= self._last_item
            and "Mission: 1" in status_msg.text
        ):
            logging.info(f"Vehicle {self.sysid}: Mission completed")
            return True
        return False


class ReachedItem(Step):
    """
    Check if a mission item is reached.

    This still relies on MISSION_ITEM_REACHED, which may not be emitted for all
    items depending on vehicle type and mission content.
    """

    def __init__(self, name: str, item: int):
        super().__init__(name)
        self._item = item

    def exec_fn(self) -> None:
        """No execution needed; just checking."""
        msg = ask_msg(
            conn=self.conn, msg_id=MsgID.MISSION_ITEM_REACHED, interval=100_000
        )
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Check if a mission item reached message was emitted."""
        msg = self.mav_manager.state.get("MISSION_ITEM_REACHED")
        if msg and msg.seq == self._item:
            logging.info(f"Vehicle {self.sysid}: ⭐ Reached item: {msg.seq}")
            msg = stop_msg(self.conn, msg_id=MsgID.MISSION_ITEM_REACHED)
            self.mav_manager.send(msg)
            return True
        return False


def make_monitoring(item_count: int) -> Action[Step]:
    """Monitor mission progress and completion."""
    name = Action.Names.MONITOR_MISSION
    monitoring = Action[Step](name=name, emoji=name.emoji)
    for item in range(1, item_count):
        monitoring.add(ReachedItem(name=f"check item {item}", item=item))
    monitoring.add(CheckEndMission(name="check end mission", item_count=item_count))
    return monitoring
