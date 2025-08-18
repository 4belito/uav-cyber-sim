"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based UAV using MAVLink. The mission file should be in `.waypoints`
format.

"""

import logging
import time
from functools import partial

from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.mission import MissionLoader
from mavlink.enums import CmdNav, MissionResult
from plan import Action, ActionNames
from plan.core import Step


def make_upload_mission(mission_path: str, from_scratch: bool = True) -> Action[Step]:
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
            f"load mission {mission_path}",
            exec_fn=partial(exec_upload_mission, mission_path=mission_path),
            check_fn=partial(check_upload_mission),
            onair=False,
        )
    )
    return upload_mission


# TODO: modularize this
def exec_upload_mission(
    conn: MAVConnection, mission_path: str = "plan/missions/simple_mission.waypoints"
):
    """Execute the upload of a mission to the UAV."""
    sysid, compid = conn.target_system, conn.target_component
    mission = MissionLoader(sysid, compid)
    count = mission.load(mission_path)
    logging.info(f"✅ Vehicle {conn.target_system}: {count} waypoints read")

    for i in range(count):
        wp = mission.item(i)
        cmd_name = CmdNav(wp.command).name
        logging.debug(
            f"🧭 Vehicle {conn.target_system}: Mission[{i}] → cmd: {cmd_name}, "
            f"x: {wp.x}, y: {wp.y}, z: {wp.z}, current: {wp.current}"
        )
    time.sleep(1)
    conn.mav.mission_count_send(sysid, compid, mission.count())
    for i in range(mission.count()):
        msg = conn.recv_match(type="MISSION_REQUEST", blocking=True, timeout=5)
        if not msg or msg.seq != i:
            raise RuntimeError(
                f"Vehicle {conn.target_system}: ❌ Unexpected mission request: {msg}"
            )
        conn.mav.send(mission.wp(i))
        logging.debug(f"✅ Vehicle {conn.target_system}: Sent mission item {i}")


def check_upload_mission(conn: MAVConnection) -> tuple[bool, None]:
    """Verify that the mission upload was successful."""
    ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    if ack and MissionResult(ack.type) == MissionResult.ACCEPTED:
        logging.info(f"✅ Vehicle {conn.target_system}: Mission upload successful!")
        return True, None
    logging.warning(f"⚠️ Mission upload failed or timed out: {ack}")
    return False, None


# Clear missions step
def exec_clear_mission(conn: MAVConnection) -> None:
    """Execute the clear mission."""
    conn.mav.mission_clear_all_send(conn.target_system, conn.target_component)


def check_clear_mission(conn: MAVConnection) -> tuple[bool, None]:
    """Verify that cleared mission was successful."""
    msg = conn.recv_match(type="STATUSTEXT")
    if msg and msg.text == "ArduPilot Ready":
        logging.info(f"🧹 Vehicle {conn.target_system}: Cleared previous mission")
        return True, None
    return False, None
