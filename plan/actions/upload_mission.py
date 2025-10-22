"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based UAV using MAVLink. The mission file should be in `.waypoints`
format.

"""

import logging
import time

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.customtypes.mission import MissionLoader
from helpers.connections.mavlink.enums import Cmd, MissionResult
from plan import Action, ActionNames
from plan.core import Step


class ClearMission(Step):
    """Step to clear previous mission from the UAV."""

    def exec_fn(self, conn: MAVConnection) -> None:
        """Execute the clear mission."""
        conn.mav.mission_clear_all_send(conn.target_system, conn.target_component)

    def check_fn(self, conn: MAVConnection) -> bool:
        """Verify that cleared mission was successful."""
        msg = conn.recv_match(type="STATUSTEXT")
        if msg and msg.text == "ArduPilot Ready":
            logging.info(f"🧹 Vehicle {conn.target_system}: Cleared previous mission")
            return True
        return False


class UploadMission(Step):
    """Step to upload a mission to the UAV."""

    def __init__(self, name: str, mission_path: str):
        super().__init__(name=name)
        self.mission_path = mission_path

    def exec_fn(self, conn: MAVConnection) -> None:
        """Execute the upload of a mission to the UAV."""
        sysid, compid = conn.target_system, conn.target_component
        mission = MissionLoader(sysid, compid)
        count = mission.load(self.mission_path)
        logging.info(f"✅ Vehicle {conn.target_system}: {count} waypoints read")

        for i in range(count):
            wp = mission.item(i)
            cmd_name = Cmd(wp.command).name
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
                    f"Vehicle {conn.target_system}: ❌ Unexpected mission request:"
                    f" {msg}"
                )
            conn.mav.send(mission.wp(i))
            logging.debug(f"✅ Vehicle {conn.target_system}: Sent mission item {i}")

    def check_fn(self, conn: MAVConnection) -> bool:
        """Verify that the mission upload was successful."""
        ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
        if ack and MissionResult(ack.type) == MissionResult.ACCEPTED:
            logging.info(f"✅ Vehicle {conn.target_system}: Mission upload successful!")
            return True
        logging.warning(f"⚠️ Mission upload failed or timed out: {ack}")
        return False


def make_upload_mission(mission_path: str, from_scratch: bool = True) -> Action[Step]:
    """Create an upload mission action."""
    name = ActionNames.UPLOAD_MISSION
    upload_mission = Action[Step](name=name, emoji=name.emoji)
    if from_scratch:
        upload_mission.add(ClearMission(name="clear previous mission"))

    upload_mission.add(UploadMission(name="upload mission", mission_path=mission_path))
    return upload_mission
