"""
Upload mission action module.

Defines the action to upload a mission from a file located in the `missions/` folder
to an ArduPilot-based vehicle using MAVLink. The mission file should be in `.waypoints`
format.

"""

import logging

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import (
    MAVLink_mission_item_int_message as ItemIntMsg,
)
from pymavlink.dialects.v20.ardupilotmega import MAVLink_mission_item_message as ItemMsg

from simulator.helpers.connections.mavlink.customtypes.mission import MissionLoader
from simulator.helpers.connections.mavlink.customtypes.vehicle_state import (
    VehicleStateP,
)
from simulator.helpers.connections.mavlink.enums import Cmd, MissionResult
from simulator.planner.action import Action
from simulator.planner.step import Step


def mission_item_to_int(
    wp: ItemMsg,
) -> ItemIntMsg:
    """Convert MISSION_ITEM to MISSION_ITEM_INT before sending to ArduPilot."""

    frame = wp.frame

    if frame == mavutil.mavlink.MAV_FRAME_GLOBAL:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT
    elif frame == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    elif frame == mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT

    is_global = frame in {
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
    }

    x = int(wp.x * 1e7) if is_global else int(wp.x)
    y = int(wp.y * 1e7) if is_global else int(wp.y)

    return mavutil.mavlink.MAVLink_mission_item_int_message(
        wp.target_system,
        wp.target_component,
        wp.seq,
        frame,
        wp.command,
        wp.current,
        wp.autocontinue,
        wp.param1,
        wp.param2,
        wp.param3,
        wp.param4,
        x,
        y,
        wp.z,
    )


def _got_request(state: VehicleStateP, seq: int) -> bool:
    """Return True if the latest MISSION_REQUEST or MISSION_REQUEST_INT matches seq."""
    req = state.get("MISSION_REQUEST_INT") or state.get("MISSION_REQUEST")
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
            self.mav_manager.state.messages.pop("MISSION_ACK", None)
            return True
        return False


class SendMissionCount(Step):
    """Step to send MISSION_COUNT and wait for the first MISSION_REQUEST (seq=0)."""

    def __init__(self, name: str, item_count: int) -> None:
        super().__init__(name=name)
        self._count = item_count

    def exec_fn(self) -> None:
        """Send MISSION_COUNT and clear stale request cache."""
        _clear_requests(self.mav_manager.state)
        self._count_msg = self.conn.mav.mission_count_encode(
            self.conn.target_system, self.conn.target_component, self._count
        )
        self.mav_manager.send(self._count_msg)

    def check_fn(self) -> bool:
        """Return True once ArduPilot requests seq=0; retry MISSION_COUNT on timeout."""
        while not _got_request(state=self.mav_manager.state, seq=0):
            self.exec_fn()
            # time.sleep(0.05)
        _clear_requests(self.mav_manager.state)
        return True


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
        msg = mission_item_to_int(mission.wp(self._seq))
        self.mav_manager.send(msg)

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
