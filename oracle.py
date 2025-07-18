"""
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
"""

from typing import cast
from pymavlink.dialects.v20 import common as mavlink2  # type: ignore
import pymavlink.dialects.v20.ardupilotmega as mavlink

from helpers.change_coordinates import GRA  # ,global2local
from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import MsgID
from mavlink.util import ask_msg
from monitor import UAVMonitor


class Oracle(UAVMonitor):
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to UAV logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self, conns: dict[int, MAVConnection], name: str = "Oracle ⚪", verbose: int = 1
    ) -> None:
        super().__init__(conns, name, verbose)

    def run(self):
        """Run the Oracle to manage UAV connections and communication."""
        if self.verbose:
            print(f"{self.name}: 🏁 Starting Oracle with {len(self.conns)} vehicles")
        for conn in self.conns.values():
            ask_msg(
                conn, self.verbose, msg_id=MsgID.GLOBAL_POSITION_INT, interval=100_000
            )

        while self.conns:
            for sysid, conn in list(self.conns.items()):
                try:
                    msg = conn.recv_msg()
                    if not msg:
                        continue
                except:
                    continue
                match msg.get_type():
                    case "GLOBAL_POSITION_INT":
                        self._get_global_pos(
                            cast(mavlink.MAVLink_global_position_int_message, msg),
                            sysid,
                        )
                    case "STATUSTEXT":
                        msg = cast(mavlink.MAVLink_statustext_message, msg)
                        if self._is_plan_done(conn, msg, sysid):
                            self.remove(sysid)
                    case "OPEN_DRONE_ID_BASIC_ID":
                        self.retransmit_remote_id(
                            cast(mavlink.MAVLink_open_drone_id_basic_id_message, msg),
                            sysid,
                        )
                    case _:
                        pass

    def retransmit_remote_id(
        self, msg: mavlink.MAVLink_open_drone_id_basic_id_message, sysid: int
    ):
        """Retransmit remote ID information for all vehicles."""
        if self.verbose > 1:
            print(f"{self.name}: 🔁 Received Open Drone ID from {sysid}")
        pos = self.pos.get(sysid, None)
        if pos is None:
            return
        for other_sysid, other_conn in self.conns.items():
            if other_sysid == sysid:
                continue
            other_pos = self.pos.get(other_sysid, None)
            if other_pos is None:
                continue
            dist = GRA.distance(pos, other_pos)
            if dist > 100:
                continue
            other_conn.mav.open_drone_id_basic_id_send(
                target_system=other_conn.target_system,
                target_component=other_conn.target_component,
                id_or_mac=msg.id_or_mac,
                id_type=msg.id_type,
                ua_type=msg.ua_type,
                uas_id=msg.uas_id,
            )
            if self.verbose > 1:
                print(
                    f"{self.name}: 🔁 Retransmitted Open Drone ID from {sysid} to {other_sysid}"
                )
