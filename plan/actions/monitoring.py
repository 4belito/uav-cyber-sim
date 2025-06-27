"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based UAV using MAVLink. The mission file should be in `.waypoints`
format.

"""

from typing import cast

import pymavlink.dialects.v20.ardupilotmega as mavlink

from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import MsgID
from mavlink.util import ask_msg, stop_msg
from plan import Action, ActionNames
from plan.core import Step


def make_monitoring() -> Action[Step]:
    """Create an upload mission action."""
    monitoring = Action[Step](name=ActionNames.UPLOAD_MISSION, emoji="📤")
    monitoring.add(
        Step(
            "monitoring",
            exec_fn=exec_monitoring,
            check_fn=check_monitoring,
            onair=False,
        )
    )
    return monitoring


def exec_monitoring(
    conn: MAVConnection,
    verbose: int,
) -> None:
    """Start monitoring the UAV by requesting periodic GLOBAL_POSITION_INT."""
    if verbose > 1:
        ask_msg(conn, verbose, msg_id=MsgID.GLOBAL_POSITION_INT, interval=100_000)


def check_monitoring(
    conn: MAVConnection,
    verbose: int,
) -> tuple[bool, None]:
    """
    Monitor the UAV mission progress by checking for waypoint reached, position,
    and mission completion messages.
    """
    msg = conn.recv_match(blocking=True, timeout=1)
    if msg:
        # ✅ Reached a waypoint
        if msg.get_type() == "MISSION_ITEM_REACHED" and verbose:
            msg = cast(mavlink.MAVLink_mission_item_reached_message, msg)
            print(f"Vehicle {conn.target_system}: 📌 Reached waypoint: {msg.seq}")

        # ✅ UAV position
        if (msg.get_type() == "GLOBAL_POSITION_INT") and (verbose > 1):
            msg = cast(mavlink.MAVLink_global_position_int_message, msg)
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(
                f"Vehicle {conn.target_system}: 📍 Position: "
                f"lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m"
            )

        # ✅ Look for end hints in text
        if msg.get_type() == "STATUSTEXT" and verbose:
            msg = cast(mavlink.MAVLink_statustext_message, msg)
            text = msg.text.strip().lower()
            if "disarming" in text:
                print(f"Vehicle {conn.target_system}: 🏁 Mission completed")
                if verbose > 1:
                    stop_msg(conn, msg_id=MsgID.GLOBAL_POSITION_INT)
                return True, None
    return False, None
