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
from mavlink.util import CustomCmd, ask_msg, get_GRA_position


class Oracle:
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to UAV logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self, conns: dict[int, MAVConnection], name: str = "Oracle ⚪", verbose: int = 1
    ) -> None:
        self.pos: dict[int, GRA] = {}
        self.conns = conns
        self.name = name
        self.verbose = verbose

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

    def remove(self, sysid: int):
        """Remove vehicles from the environment."""
        del self.conns[sysid]
        del self.pos[sysid]

    def gather_broadcasts(self):
        """Collect and store broadcasts (global positions so far) from all vehicles."""
        for sysid in self.conns:
            self.get_global_pos(sysid)

    def get_global_pos(self, sysid: int):
        """Get the current global position of the specified vehicle."""
        msg = self.conns[sysid].recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=0.001
        )
        if not msg:
            return None
        self._get_global_pos(
            msg,
            sysid,
        )

    def _get_global_pos(
        self, msg: mavlink.MAVLink_global_position_int_message, sysid: int
    ):
        """Get the current global position of the specified vehicle."""
        self.pos[sysid] = get_GRA_position(
            msg,
            sysid,
            verbose=self.verbose,
        )

    # def update_neighbors(self, sysid: int):
    #     # update this tu use mavconnecions and probably custom mavlin messages
    #     neigh_vehs = []
    #     neigh_poss = []
    #     neigh_dists = []
    #     for other, other_pos in self.pos.items():
    #         if other is veh:
    #             continue

    #         dist = np.linalg.norm(
    #             np.array([x - y for x, y in zip(other_pos, self.pos[veh.sysid])])
    #         )
    #         if dist < veh.radar_radius:
    #             neigh_vehs.append(other)
    #             neigh_poss.append(other_pos)  # this is a reference to the array
    #             neigh_dists.append(dist)

    #     # Perform transformation only on the selected ones
    #     if neigh_poss:
    #         neigh_poss = global2local(np.stack(neigh_poss), veh.home)
    #         neigh_dists = np.array(neigh_dists)

    #     veh.neighbors = Neighbors(
    #         neigh_vehs,
    #         distances=neigh_dists,  # avoid np.stack if 1D
    #         positions=neigh_poss,
    #     )

    def is_plan_done(self, sysid: int) -> bool:
        """Listen for a STATUSTEXT("DONE") message and respond with COMMAND_ACK."""
        conn = self.conns[sysid]
        msg = conn.recv_match(type="STATUSTEXT", blocking=False)
        return bool(msg and self._is_plan_done(conn, msg, sysid))

    def _is_plan_done(
        self, conn: MAVConnection, msg: mavlink.MAVLink_statustext_message, sysid: int
    ) -> bool:
        """Check for a STATUSTEXT("DONE") message and respond with COMMAND_ACK."""
        if msg.text == "DONE":
            conn.mav.command_ack_send(
                command=CustomCmd.PLAN_DONE, result=mavlink2.MAV_RESULT_ACCEPTED
            )
            if self.verbose:
                print(f"{self.name}: ✅ Vehicle {sysid} completed its mission")
            return True
        return False

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
