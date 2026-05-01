"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based vehicle using MAVLink. The mission file should be in `.waypoints`
format.

"""

import logging
import time

from simulator.helpers.connections.mavlink.customtypes.mission import MissionLoader
from simulator.helpers.connections.mavlink.customtypes.vehicle_state import (
    VehicleStateP,
)
from simulator.helpers.connections.mavlink.enums import Cmd, MissionResult
from simulator.planner.action import Action
from simulator.planner.step import Step

_REQUEST_TIMEOUT = 2.0
_MAX_COUNT_RETRIES = 5


def _got_request(state: VehicleStateP, seq: int) -> bool:
    """Return True if the latest MISSION_REQUEST or MISSION_REQUEST_INT matches seq."""
    req = state.get("MISSION_REQUEST")
    if req is not None and req.seq == seq:
        return True
    req = state.get("MISSION_REQUEST_INT")
    return req is not None and req.seq == seq


def _clear_requests(state: VehicleStateP) -> None:
    """Remove stale MISSION_REQUEST entries from the messages cache."""
    state.messages.pop("MISSION_REQUEST", None)
    state.messages.pop("MISSION_REQUEST_INT", None)


class ClearMission(Step):
    """Step to clear previous mission from the vehicle."""

    def exec_fn(self) -> None:
        """Execute the clear mission."""
        self.mav_manager.state.messages.pop("MISSION_ACK", None)
        msg = self.conn.mav.mission_clear_all_encode(
            self.conn.target_system, self.conn.target_component
        )
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Verify that cleared mission was successful."""
        ack = self.mav_manager.state.get("MISSION_ACK")
        if ack and MissionResult(ack.type) == MissionResult.ACCEPTED:
            logging.info(f"🧹 Vehicle {self.sysid}: Cleared previous mission")
            return True
        return False


class SendMissionCount(Step):
    """Step to send MISSION_COUNT and wait for the first MISSION_REQUEST (seq=0)."""

    def __init__(self, name: str, item_count: int) -> None:
        super().__init__(name=name)
        self._count = item_count
        self._retries = 0
        self._last_sent: float = 0.0
        self._count_msg: object = None

    def exec_fn(self) -> None:
        """Send MISSION_COUNT and clear stale request cache."""
        _clear_requests(self.mav_manager.state)
        self._count_msg = self.conn.mav.mission_count_encode(
            self.conn.target_system, self.conn.target_component, self._count
        )
        self.mav_manager.send(self._count_msg)
        self._last_sent = time.time()
        self._retries = 0

    def check_fn(self) -> bool:
        """Return True once ArduPilot requests seq=0; retry MISSION_COUNT on timeout."""
        if _got_request(self.mav_manager.state, 0):
            return True
        if time.time() - self._last_sent > _REQUEST_TIMEOUT:
            if self._retries >= _MAX_COUNT_RETRIES:
                raise RuntimeError(
                    f"Vehicle {self.sysid}: MISSION_COUNT: no response after "
                    f"{_MAX_COUNT_RETRIES} retries"
                )
            self._retries += 1
            logging.warning(
                f"Vehicle {self.sysid}: MISSION_COUNT timeout, "
                f"retry {self._retries}/{_MAX_COUNT_RETRIES}"
            )
            _clear_requests(self.mav_manager.state)
            self.mav_manager.send(self._count_msg)
            self._last_sent = time.time()
        return False


class SendMissionItem(Step):
    """Step to send one mission waypoint and wait for the next MISSION_REQUEST."""

    def __init__(self, name: str, seq: int, mission_path: str, is_last: bool) -> None:
        super().__init__(name=name)
        self._seq = seq
        self._mission_path = mission_path
        self._is_last = is_last

    def exec_fn(self) -> None:
        """Clear stale request cache, then send the waypoint for this seq."""
        _clear_requests(self.mav_manager.state)
        mission = MissionLoader(self.conn.target_system, self.conn.target_component)
        mission.load(self._mission_path)
        self.mav_manager.send(mission.wp(self._seq))

        wp = mission.item(self._seq)
        cmd_name = Cmd(wp.command).name
        logging.debug(
            f"🧭 Vehicle {self.sysid}: Mission[{self._seq}] → cmd: {cmd_name}, "
            f"x: {wp.x}, y: {wp.y}, z: {wp.z}, current: {wp.current}"
        )

    def check_fn(self) -> bool:
        """
        Return True once ArduPilot requests the next seq (or accept the mission if
        last).
        """
        if self._is_last:
            ack = self.mav_manager.state.wait_for("MISSION_ACK", timeout=5.0)
            if ack and MissionResult(ack.type) == MissionResult.ACCEPTED:
                logging.info(f"✅ Vehicle {self.sysid}: Mission successfully loaded!")

                return True
            return False
        return _got_request(self.mav_manager.state, self._seq + 1)


def make_upload_mission(mission_path: str, from_scratch: bool = True) -> Action[Step]:
    """Create an upload mission action."""
    name = Action.Names.UPLOAD_MISSION
    item_count = MissionLoader().load(mission_path)
    upload_mission = Action[Step](name=name, emoji=name.emoji)
    if from_scratch:
        upload_mission.add(ClearMission(name="clear previous mission"))

    upload_mission.add(
        SendMissionCount(name="send mission count", item_count=item_count)
    )
    for i in range(item_count):
        upload_mission.add(
            SendMissionItem(
                name=f"send item {i}",
                seq=i,
                mission_path=mission_path,
                is_last=(i == item_count - 1),
            )
        )
    return upload_mission
