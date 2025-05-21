"""
This module defines the Oracle and GCS classes, which handle UAV-to-UAV coordination,
global position tracking, and end-of-plan signaling during MAVLink-based simulations.
"""

from typing import Dict, List


from pymavlink.dialects.v20 import common as mavlink2
from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import BasePort
from helpers.change_coordinates import Position, local2global_pos  # ,global2local
from helpers.mavlink import CustomCmd, MAVConnection
from plan.actions import get_local_position

# from vehicle_logic import Neighbors, VehicleLogic


offsets = [
    (-4.891311895481473, -1.3721148432366883, 0, 78),
    (4.272452965244035, -2.0624885894963674, 0, 308),
    (3.0267596011325804, 2.2013367285195207, 0, 21),
    (-2.0396000412483604, 0.07477394309030139, 0, 322),
    (-3.3237962685317655, 0.6696946809570994, 0, 289),
    (1.5219370395388898, -1.1094923308348625, 0, 149),
    (0.03170515670252261, 1.4467355333174172, 0, 17),
    (-3.167665911861357, -3.81136282344402, 0, 22),
    (-2.444501492007306, 2.3741511359188525, 0, 349),
    (0.48442199707763045, 4.043149308497993, 0, 38),
]
homes = [offset[:3] for offset in offsets]


##################################


class Oracle:
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to UAV logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self, sysids: List[int], name: str = "Oracle ⚪", base_port: int = BasePort.ORC
    ) -> None:
        self.pos: Dict[int, Position] = {}
        self.conns: Dict[int, MAVConnection] = {}
        for sysid in sysids:
            port = base_port + 10 * (sysid - 1)
            conn: MAVConnection = connect(f"udp:127.0.0.1:{port}")  # type: ignore
            conn.wait_heartbeat()
            self.conns[sysid] = conn
            print(f"🔗 UAV logic {sysid} is connected to {name}")

    def remove(self, sysid: int):
        """
        Remove vehicles from the enviroment
        """
        del self.conns[sysid]

    def gather_broadcasts(self):
        """
        Collect and store boradcasts (global positions sofar) from all vehicles.
        """
        for sysid in self.conns:
            pos = self.get_global_pos(sysid)
            if pos is not None:
                self.pos[sysid] = pos

    def get_global_pos(self, sysid: int):
        """
        Get the current global position of the specified vehicle.
        """
        pos = get_local_position(self.conns[sysid])
        if pos is not None:
            pos = local2global_pos(pos, homes[sysid - 1])
        return pos

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
        """
        Listen for a STATUSTEXT("DONE") message and respond with COMMAND_ACK.
        """
        conn = self.conns[sysid]
        msg = conn.recv_match(type="STATUSTEXT", blocking=False)
        if msg and msg.text == "DONE":
            conn.mav.command_ack_send(
                command=CustomCmd.PLAN_DONE, result=mavlink2.MAV_RESULT_ACCEPTED
            )
            print(f"✅ Vehicle {sysid} terminated")
            return True
        return False


class GCS(Oracle):
    """
    Ground Control Station class extending Oracle with trajectory logging.
    """

    def __init__(self, sysids: List[int], name: str = "blue"):
        self.name = name
        super().__init__(sysids, name=f"GCS {name}", base_port=BasePort.GCS)
        self.paths: Dict[int, List[Position]] = {sysid: [] for sysid in sysids}

    def save_pos(self):
        """
        Save the current global position of each UAV to their trajectory path.
        """
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)
