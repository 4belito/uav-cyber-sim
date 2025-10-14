"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based UAV using MAVLink. The mission file should be in `.waypoints`
format.

"""

import logging

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import MsgID
from helpers.connections.mavlink.streams import stop_msg
from helpers.coordinates import GRA
from plan import Action, ActionNames
from plan.core import Step


def make_monitoring(
    gra_origin: GRA,
    items: list[int] = [],
) -> Action[Step]:
    """Monitor mission items."""
    monitoring = Action[Step](name=ActionNames.MONITOR_MISSION, emoji="👁️")

    class CheckItem(Step):
        def __init__(self, name: str, emoji: str = "🔹", item: int = 0):
            super().__init__(name, emoji)
            self.item = item

        def exec_fn(self, conn: MAVConnection) -> None:
            """No execution needed; just checking."""
            pass

        def check_fn(self, conn: MAVConnection) -> bool:
            """Check if a item is reached."""
            msg = conn.recv_match(type="MISSION_ITEM_REACHED")
            # logging.debug(f"Vehicle {conn.target_system}: MISSION_ITEM_REACHED: {msg}")
            if msg:
                if msg.seq == self.item:
                    logging.info(
                        f"Vehicle {conn.target_system}: ⭐ Reached item: {msg.seq}"
                    )
                    return True
            return False

    class NextWaypoint(Step):
        def __init__(self, name: str, emoji: str = "📌", item_seq: int = 1):
            super().__init__(name, emoji)
            self.item_seq = item_seq

        def exec_fn(self, conn: MAVConnection) -> None:
            """Request the next waypoint from the UAV."""
            exec_next_waypoint(conn, self.item_seq)

        def check_fn(self, conn: MAVConnection) -> bool:
            """Check the next waypoint from the UAV."""
            item = conn.recv_match(type="MISSION_ITEM")
            if not item:
                return False
            logging.debug(f"Vehicle {conn.target_system}: MISSION_ITEM: {item}")
            gra_wp = GRA(lat=item.x, lon=item.y, alt=item.z)  # type: ignore
            self.target_pos = gra_origin.to_rel(gra_wp)
            logging.info(
                f"Vehicle {conn.target_system}: 📍 Target Position: {self.target_pos}"
            )
            return True

    monitoring.add(NextWaypoint(name=f"next waypoint {items[0]}", item_seq=items[0]))
    for i in range(1, len(items) - 1):
        monitoring.add(CheckItem(name=f"check item {i}", item=items[i]))
        monitoring.add(
            NextWaypoint(name=f"next waypoint {items[i + 1]}", item_seq=items[i + 1])
        )

    class CheckEndMission(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """Start monitoring the UAV by requesting periodic GLOBAL_POSITION_INT."""
            pass

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

    monitoring.add(CheckEndMission(name="check end mission"))
    return monitoring


def exec_next_waypoint(conn: MAVConnection, item_seq: int = 1) -> None:
    """Request the next waypoint from the UAV."""
    conn.mav.mission_request_send(conn.target_system, conn.target_component, item_seq)
