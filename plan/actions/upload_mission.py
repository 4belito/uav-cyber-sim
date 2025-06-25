"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based UAV using MAVLink. The mission file should be in `.waypoints`
format.

"""

import time
from functools import partial
from typing import cast

import pymavlink.dialects.v20.ardupilotmega as mavlink

from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.mission import MissionLoader
from mavlink.enums import CmdNav, MissionResult
from plan import Action, ActionNames
from plan.core import Step


def make_upload_mission(mission_name: str, from_scratch: bool = True) -> Action[Step]:
    """Create an upload mission action."""
    upload_mission = Action[Step](name=ActionNames.UPLOAD_MISSION, emoji="📤")
    if from_scratch:
        upload_mission.add(
            Step(
                "clear uav missions",
                exec_fn=partial(exec_clear_mission),
                check_fn=partial(check_clear_mission),
                onair=False,
            )
        )
    upload_mission.add(
        Step(
            f"load mission {mission_name}",
            exec_fn=partial(exec_upload_mission, mission_name=mission_name),
            check_fn=partial(check_upload_mission),
            onair=False,
        )
    )
    return upload_mission


# TODO: modularize this
def exec_upload_mission(
    conn: MAVConnection,
    verbose: int = 1,
    mission_name: str = "mission",
):
    """Execute the upload of a mission to the UAV."""
    sysid, compid = conn.target_system, conn.target_component
    mission = MissionLoader(sysid, compid)
    count = mission.load(f"plan/missions/{mission_name}.waypoints")
    if verbose:
        print(f"✅ {count} waypoints read.")
    if verbose == 2:
        for i in range(count):
            wp = mission.item(i)
            cmd_name = CmdNav(wp.command).name
            print(
                f"🧭 Mission[{i}] → cmd: {cmd_name}, "
                f"x: {wp.x}, y: {wp.y}, z: {wp.z}, current: {wp.current}"
            )
    time.sleep(1)
    conn.mav.mission_count_send(sysid, compid, mission.count())
    for i in range(mission.count()):
        msg = conn.recv_match(type="MISSION_REQUEST", blocking=True, timeout=5)
        if not msg or msg.seq != i:
            raise RuntimeError(f"❌ Unexpected mission request: {msg}")
        conn.mav.send(mission.wp(i))
        if verbose:
            print(f"✅ Sent mission item {i}")


def check_upload_mission(
    conn: MAVConnection,
    verbose: int,
) -> tuple[bool, None]:
    """Verify that the mission upload was successful."""
    ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    if ack and MissionResult(ack.type) == MissionResult.ACCEPTED:
        if verbose:
            print("🎉 Mission upload successful!")
        return True, None
    if verbose:
        print(f"⚠️ Mission upload failed or timed out: {ack}")
    return False, None


# Clear missioins step
def exec_clear_mission(
    conn: MAVConnection,
    verbose: int = 1,
):
    """Execute the clear mission."""
    conn.mav.mission_clear_all_send(conn.target_system, conn.target_component)


def check_clear_mission(
    conn: MAVConnection,
    verbose: int,
) -> tuple[bool, None]:
    """Verify that cleared mission was succesful."""
    msg = conn.recv_match(type="STATUSTEXT")
    if msg and msg.text == "ArduPilot Ready":
        if verbose == 2:
            print("🧹 Cleared previous mission")
        return True, None
    return False, None


def check_monitoring(
    conn: MAVConnection,
    verbose: int,
    n_items: int = 7,
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
            print(f"📌 Reached waypoint: {msg.seq}")
            if msg.seq == n_items - 1:
                print("✅ Final waypoint reached")

        # ✅ UAV position
        elif msg.get_type() == "GLOBAL_POSITION_INT" and verbose > 1:
            msg = cast(mavlink.MAVLink_global_position_int_message, msg)
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(f"📍 Position: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m")

        # ✅ Look for end hints in text
        elif msg.get_type() == "STATUSTEXT":
            msg = cast(mavlink.MAVLink_statustext_message, msg)
            text = msg.text.strip().lower()
            if "disarming" in text:
                print("🏁 Mission completed")
                return True, None
    return False, None
