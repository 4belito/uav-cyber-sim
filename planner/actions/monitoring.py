"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based UAV using MAVLink. The mission file should be in `.waypoints`
format.

"""

import logging

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import MsgID
from helpers.connections.mavlink.streams import ask_msg, stop_msg
from helpers.coordinates import GRA
from planner.action import Action
from planner.step import Step


class CheckItems(Step):
    """Request and check all waypoints from the UAV."""

    def __init__(self, name: str):
        super().__init__(name)
        self._item_seq = 0
        self._mission_count: int | None = None

    def exec_fn(self, conn: MAVConnection) -> None:
        """Request the next waypoint from the UAV."""
        ask_msg(conn, msg_id=MsgID.MISSION_CURRENT, interval=100_000)
        conn.mav.mission_request_list_send(conn.target_system, conn.target_component)

    def check_fn(self, conn: MAVConnection) -> bool:
        """Check the next waypoint from the UAV."""
        if not self._mission_count:
            msg = conn.recv_match(type="MISSION_COUNT")
            if msg:
                self._mission_count = msg.count
                logging.info(
                    f"📦 Vehicle {conn.target_system} has {msg.count} mission items"
                )
            else:
                return False

        curr_msg = conn.recv_match(type="MISSION_CURRENT")
        if not curr_msg or curr_msg.seq == self._item_seq:
            return False
        while self._item_seq < curr_msg.seq:
            logging.info(
                f"Vehicle {conn.target_system}: ⭐ Reached item: {self._item_seq}"
            )
            self._item_seq += 1
        conn.mav.mission_request_send(
            conn.target_system, conn.target_component, self._item_seq
        )
        item = conn.recv_match(type="MISSION_ITEM", blocking=True)
        gra_wp = GRA(lat=float(item.x), lon=float(item.y), alt=float(item.z))  # type: ignore
        self.target_pos = self.origin.to_rel(gra_wp)
        logging.info(
            f"Vehicle {conn.target_system}: 📍 Target Position:"
            f" {self.target_pos.short()}"
        )
        if self._item_seq == self._mission_count - 1:
            return True
        return False


class CheckEndMission(Step):
    """Check for mission completion."""

    def exec_fn(self, conn: MAVConnection) -> None:
        """No execution needed; just checking."""
        return

    def check_fn(self, conn: MAVConnection) -> bool:
        """Check mission completion."""
        msg = conn.recv_match(type="STATUSTEXT")
        if msg:
            text = msg.text.strip().lower()
            if "disarming" in text:
                logging.info(f"Vehicle {conn.target_system}: Mission completed")
                stop_msg(conn, msg_id=MsgID.GLOBAL_POSITION_INT)
                return True
        return False


def make_monitoring() -> Action[Step]:
    """Monitor mission items."""
    name = Action.Names.MONITOR_MISSION
    monitoring = Action[Step](name=name, emoji=name.emoji)
    monitoring.add(CheckItems(name="check items"))
    monitoring.add(CheckEndMission(name="check end mission"))
    return monitoring


class ReachedItem(Step):
    """
    Check if a mission item is reached.
    Ardupilot does not send MISSION_ITEM_REACHED for all mission items.
    CheckItems is more reliable for monitoring mission progress.
    """

    def __init__(self, name: str, item: int = 0):
        super().__init__(name)
        self._item = item

    def exec_fn(self, conn: MAVConnection) -> None:
        """No execution needed; just checking."""
        ask_msg(conn=conn, msg_id=MsgID.GLOBAL_POSITION_INT, interval=100_000)

    def check_fn(self, conn: MAVConnection) -> bool:
        """Check if a item is reached."""
        msg = conn.recv_match(type="MISSION_ITEM_REACHED")
        # logging.debug(f"Vehicle {conn.target_system}: MISSION_ITEM_REACHED: {msg}")
        if msg:
            if msg.seq == self._item:
                logging.info(
                    f"Vehicle {conn.target_system}: ⭐ Reached item: {msg.seq}"
                )
                return True
        return False
