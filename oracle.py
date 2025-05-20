"""
This module defines the Oracle and GCS classes, which handle UAV-to-UAV coordination,
global position tracking, and end-of-plan signaling during MAVLink-based simulations.
"""

from typing import Dict, List

import numpy as np
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink.mavutil import mavlink_connection as connect  # type: ignore

from config import BasePort
from helpers.change_coordinates import Position, local2global_pos  # ,global2local
from helpers.mavlink import CustomCmd, MAVConnection
from plan.actions import get_local_position

# from vehicle_logic import Neighbors, VehicleLogic

### Hardcoded for now as part of a step-by-step development process
offsets = [  # east, north, up, heading
    (5, 5, 0, 90),
    (10, 0, 0, 45),
    (-5, -10, 0, 225),
    (-15, 0, 0, 0),
    (0, -20, 0, 0),
]
homes = np.array([offset[:3] for offset in offsets])


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
