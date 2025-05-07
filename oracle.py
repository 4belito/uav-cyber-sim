import numpy as np
from pymavlink import mavutil
from helpers.change_coordinates import local2global, global2local
from vehicle_logic import VehicleLogic, Neighbors
from typing import List
from config import GCS_BASE_PORT
from plan.actions import get_local_position
from pymavlink.dialects.v20 import common as mavlink2

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
    Oracle class for veh-veh communication and general simulation controls
    """

    def __init__(self, sysids: List[int]) -> None:
        self.sysids = sysids
        self.pos = {}
        self.conns = {}
        for sysid in sysids:
            cs_port = GCS_BASE_PORT + 10 * (sysid - 1)
            conn = mavutil.mavlink_connection(f"udp:127.0.0.1:{cs_port}")
            conn.wait_heartbeat()
            self.conns[sysid] = conn

    def remove(self, sysid: int):
        """
        Remove vehicles from the enviroment
        """
        del self.sysids[sysid]
        del self.conns[sysid]

    def gather_broadcasts(self):
        for sysid in self.sysids:
            pos = self.get_global_pos(sysid)
            if pos is not None:
                self.pos[sysid] = pos

    def get_global_pos(self, sysid):
        pos = get_local_position(self.conns[sysid])
        if pos is not None:
            pos = local2global(pos, homes[sysid - 1])
        return pos

    def update_neighbors(self, veh: VehicleLogic):
        # TODO update this tu use mavconnecions and probably custom mavlin messages
        neigh_vehs = []
        neigh_poss = []
        neigh_dists = []
        for other, other_pos in self.pos.items():
            if other is veh:
                continue

            dist = np.linalg.norm(other_pos - self.pos[veh])
            if dist < veh.radar_radius:
                neigh_vehs.append(other)
                neigh_poss.append(other_pos)  # this is a reference to the array
                neigh_dists.append(dist)

        # Perform transformation only on the selected ones
        if neigh_poss:
            neigh_poss = global2local(np.stack(neigh_poss), veh.home)
            neigh_dists = np.array(neigh_dists)

        veh.neighbors = Neighbors(
            neigh_vehs,
            distances=neigh_dists,  # avoid np.stack if 1D
            positions=neigh_poss,
        )

    def is_plan_done(self, sysid):
        msg = self.conns[sysid].recv_match(blocking=True)
        print(msg)
        return msg and msg.get_type() == "STATUSTEXT" and msg.text == "DONE"


class GCS(Oracle):
    def __init__(self, sysids: List[int]):
        super().__init__(sysids)
        self.paths = {sysid: [] for sysid in sysids}

    def save_pos(self):
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)
