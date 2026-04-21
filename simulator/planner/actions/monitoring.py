"""
Upload mission action module.

Defines mission monitoring helpers for ArduPilot-based vehicles.
This version is safer for ArduPlane because MISSION_CURRENT is treated as the
currently active item, not as proof that prior items were physically reached.
"""

import logging

from simulator.helpers.connections.mavlink.enums import MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.helpers.coordinates import GRA
from simulator.planner.action import Action
from simulator.planner.step import Step


class CheckItems(Step):
    """Monitor the active mission item and expose its target position."""

    def __init__(self, name: str):
        super().__init__(name)
        self._mission_count: int | None = None
        self._active_seq: int | None = None
        self._last_logged_seq: int | None = None
        self._requested_seq: int | None = None

    def exec_fn(self) -> None:
        """Start streaming mission progress and request mission list."""
        msg = ask_msg(self.conn, msg_id=MsgID.MISSION_CURRENT, interval=100_000)
        self.mav_manager.send(msg)

        # Useful for end-of-mission checks.
        try:
            msg = ask_msg(self.conn, msg_id=MsgID.EXTENDED_SYS_STATE, interval=100_000)
            self.mav_manager.send(msg)
        except Exception:
            pass

        msg = self.conn.mav.mission_request_list_encode(
            self.conn.target_system,
            self.conn.target_component,
        )
        self.mav_manager.send(msg)

    def _request_item(self, seq: int) -> None:
        """Request a specific mission item from the vehicle."""
        msg = self.conn.mav.mission_request_encode(
            self.conn.target_system,
            self.conn.target_component,
            seq,
        )
        self.mav_manager.send(msg)
        self._requested_seq = seq

    def _get_requested_item(self):
        """Return the most recently requested mission item."""
        item = self.mav_manager.state.get("MISSION_ITEM")
        if item and getattr(item, "seq", None) == self._requested_seq:
            return item

        item_int = self.mav_manager.state.get("MISSION_ITEM_INT")
        if item_int and getattr(item_int, "seq", None) == self._requested_seq:
            return item_int

        return None

    def _extract_target_position(self, item) -> None:
        """Convert mission item coordinates into local target position."""
        if item.get_type() == "MISSION_ITEM_INT":
            lat = float(item.x) / 1e7
            lon = float(item.y) / 1e7
            alt = float(item.z)
        else:
            lat = float(item.x)
            lon = float(item.y)
            alt = float(item.z)

        gra_wp = GRA(lat=lat, lon=lon, alt=alt)
        self.target_pos = self.origin.to_rel(gra_wp)
        logging.info(
            f"Vehicle {self.sysid}: 📍 Active Target Position: "
            f"{self.target_pos.short()}"
        )

    def check_fn(self) -> bool:
        """Track the active mission item. Finish when the last item becomes active."""
        if self._mission_count is None:
            msg = self.mav_manager.state.get("MISSION_COUNT")
            if not msg:
                return False
            self._mission_count = msg.count
            logging.info(
                f"📦 Vehicle {self.sysid} has {self._mission_count} mission items"
            )
            if self._mission_count <= 0:
                return False

        curr_msg = self.mav_manager.state.get("MISSION_CURRENT")
        if not curr_msg:
            return False

        curr_seq = curr_msg.seq
        if self._active_seq != curr_seq:
            self._active_seq = curr_seq

            if self._last_logged_seq != curr_seq:
                logging.info(
                    f"Vehicle {self.sysid}: 🧭 Active mission item: {curr_seq}"
                )
                self._last_logged_seq = curr_seq

            if 0 <= curr_seq < self._mission_count:
                self._request_item(curr_seq)
                item = self.mav_manager.state.wait_for("MISSION_ITEM", timeout=1.0)
                if not item or getattr(item, "seq", None) != curr_seq:
                    item = self.mav_manager.state.wait_for(
                        "MISSION_ITEM_INT", timeout=1.0
                    )
                if (
                    item
                    and getattr(
                        item,
                        "seq",
                        None,
                    )
                    == curr_seq
                ):
                    self._extract_target_position(item)

        # For ArduPlane, this means the final mission item is now active.
        # Actual mission completion is handled by CheckEndMission.
        return self._mission_count is not None and curr_seq >= self._mission_count - 1


class CheckEndMission(Step):
    """Check for mission completion after the last mission item becomes active."""

    def __init__(self, name: str):
        super().__init__(name)
        self._saw_final_item = False

    def exec_fn(self) -> None:
        """Request messages useful for end-of-mission detection."""
        try:
            msg = ask_msg(self.conn, msg_id=MsgID.EXTENDED_SYS_STATE, interval=100_000)
            self.mav_manager.send(msg)
        except Exception:
            pass

        try:
            msg = ask_msg(self.conn, msg_id=MsgID.STATUSTEXT, interval=100_000)
            self.mav_manager.send(msg)
        except Exception:
            pass

        try:
            msg = ask_msg(self.conn, msg_id=MsgID.MISSION_CURRENT, interval=100_000)
            self.mav_manager.send(msg)
        except Exception:
            pass

    def check_fn(self) -> bool:
        """Detect mission completion robustly for Plane/Copter."""
        count_msg = self.mav_manager.state.get("MISSION_COUNT")
        curr_msg = self.mav_manager.state.get("MISSION_CURRENT")

        if count_msg and curr_msg and curr_msg.seq >= count_msg.count - 1:
            self._saw_final_item = True

        if not self._saw_final_item:
            return False

        ext = self.mav_manager.state.get("EXTENDED_SYS_STATE")
        if ext and getattr(ext, "landed_state", None) == 1:
            logging.info(f"Vehicle {self.sysid}: Mission completed (landed)")
            try:
                msg = stop_msg(self.conn, msg_id=MsgID.MISSION_CURRENT)
                self.mav_manager.send(msg)
            except Exception:
                pass
            try:
                msg = stop_msg(self.conn, msg_id=MsgID.EXTENDED_SYS_STATE)
                self.mav_manager.send(msg)
            except Exception:
                pass
            return True

        msg = self.mav_manager.state.get("STATUSTEXT")
        if msg:
            text = msg.text.strip().lower()
            if (
                "disarming" in text
                or "land complete" in text
                or "mission complete" in text
            ):
                logging.info(f"Vehicle {self.sysid}: Mission completed")
                try:
                    msg = stop_msg(self.conn, msg_id=MsgID.MISSION_CURRENT)
                    self.mav_manager.send(msg)
                except Exception:
                    pass
                try:
                    msg = stop_msg(self.conn, msg_id=MsgID.EXTENDED_SYS_STATE)
                    self.mav_manager.send(msg)
                except Exception:
                    pass
                return True

        return False


def make_monitoring() -> Action[Step]:
    """Monitor mission progress and completion."""
    name = Action.Names.MONITOR_MISSION
    monitoring = Action[Step](name=name, emoji=name.emoji)
    monitoring.add(CheckItems(name="check items"))
    monitoring.add(CheckEndMission(name="check end mission"))
    return monitoring


class ReachedItem(Step):
    """
    Check if a mission item is reached.

    This still relies on MISSION_ITEM_REACHED, which may not be emitted for all
    items depending on vehicle type and mission content.
    """

    def __init__(self, name: str, item: int = 0):
        super().__init__(name)
        self._item = item

    def exec_fn(self) -> None:
        """No execution needed; just checking."""
        msg = ask_msg(
            conn=self.conn, msg_id=MsgID.GLOBAL_POSITION_INT, interval=100_000
        )
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Check if a mission item reached message was emitted."""
        msg = self.mav_manager.state.get("MISSION_ITEM_REACHED")
        if msg and msg.seq == self._item:
            logging.info(f"Vehicle {self.sysid}: ⭐ Reached item: {msg.seq}")
            return True
        return False
