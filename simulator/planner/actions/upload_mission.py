"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based UAV using MAVLink. The mission file should be in `.waypoints`
format.

"""

import logging
import time

from simulator.helpers.connections.mavlink.customtypes.mission import MissionLoader
from simulator.helpers.connections.mavlink.enums import Cmd, MissionResult
from simulator.planner.action import Action
from simulator.planner.step import Step


class ClearMission(Step):
    """Step to clear previous mission from the UAV."""

    def exec_fn(self) -> None:
        """Execute the clear mission."""
        self.conn.mav.mission_clear_all_send(
            self.conn.target_system, self.conn.target_component
        )

    def check_fn(self) -> bool:
        """Verify that cleared mission was successful."""
        msg = self.conn.recv_match(type="STATUSTEXT")
        if msg and msg.text == "ArduPilot Ready":
            logging.info(
                f"🧹 Vehicle {self.conn.target_system}: Cleared previous mission"
            )
            return True
        return False


class UploadMission(Step):
    """Step to upload a mission to the UAV."""

    def __init__(self, name: str, mission_path: str):
        super().__init__(name=name)
        self.mission_path = mission_path

    def exec_fn(self) -> None:
        """Execute the upload of a mission to the UAV."""
        sysid, compid = self.conn.target_system, self.conn.target_component
        mission = MissionLoader(sysid, compid)
        count = mission.load(self.mission_path)
        logging.info(f"✅ Vehicle {self.conn.target_system}: {count} waypoints read")

        for i in range(count):
            wp = mission.item(i)
            cmd_name = Cmd(wp.command).name
            logging.debug(
                f"🧭 Vehicle {self.conn.target_system}: Mission[{i}] → cmd: {cmd_name}, "
                f"x: {wp.x}, y: {wp.y}, z: {wp.z}, current: {wp.current}"
            )
        time.sleep(1)
        self.conn.mav.mission_count_send(sysid, compid, mission.count())
        for i in range(mission.count()):
            msg = self.conn.recv_match(type="MISSION_REQUEST", blocking=True, timeout=5)
            if not msg or msg.seq != i:
                raise RuntimeError(
                    f"Vehicle {self.conn.target_system}: ❌ Unexpected mission request: {msg}"
                )
            self.conn.mav.send(mission.wp(i))
            logging.debug(
                f"✅ Vehicle {self.conn.target_system}: Sent mission item {i}"
            )

    def check_fn(self) -> bool:
        """Verify that the mission upload was successful."""
        ack = self.conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
        if ack and MissionResult(ack.type) == MissionResult.ACCEPTED:
            logging.info(
                f"✅ Vehicle {self.conn.target_system}: Mission upload successful!"
            )
            return True
        logging.warning(f"⚠️ Mission upload failed or timed out: {ack}")
        return False


def make_upload_mission(mission_path: str, from_scratch: bool = True) -> Action[Step]:
    """Create an upload mission action."""
    name = Action.Names.UPLOAD_MISSION
    upload_mission = Action[Step](name=name, emoji=name.emoji)
    if from_scratch:
        upload_mission.add(ClearMission(name="clear previous mission"))

    upload_mission.add(UploadMission(name="upload mission", mission_path=mission_path))
    return upload_mission
