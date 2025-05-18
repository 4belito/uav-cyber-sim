from typing import Dict, List

import numpy as np
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

from config import BasePort
from helpers.change_coordinates import Position, global2local, local2global_pos
from helpers.mavlink import MAVConnection
from plan.actions import get_local_position
from vehicle_logic import Neighbors, VehicleLogic

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

    def __init__(
        self, sysids: List[int], name: str = "Oracle ⚪", base_port: int = BasePort.ORC
    ) -> None:
        self.pos: Dict[int, Position] = {}
        self.conns: Dict[int, MAVConnection] = {}
        for sysid in sysids:
            port = base_port + 10 * (sysid - 1)
            conn: MAVConnection = mavutil.mavlink_connection(f"udp:127.0.0.1:{port}")  # type: ignore
            conn.wait_heartbeat()
            self.conns[sysid] = conn
            print(f"🔗 UAV logic {sysid} is connected to {name}")

    def remove(self, sysid: int):
        """
        Remove vehicles from the enviroment
        """
        del self.conns[sysid]

    def gather_broadcasts(self):
        for sysid in self.conns:
            pos = self.get_global_pos(sysid)
            if pos is not None:
                self.pos[sysid] = pos

    def get_global_pos(self, sysid: int):
        pos = get_local_position(self.conns[sysid])
        if pos is not None:
            pos = local2global_pos(pos, homes[sysid - 1])
        return pos

    def update_neighbors(self, sysid: int):
        # update this tu use mavconnecions and probably custom mavlin messages
        neigh_vehs = []
        neigh_poss = []
        neigh_dists = []
        for other, other_pos in self.pos.items():
            if other is veh:
                continue

            dist = np.linalg.norm(
                np.array([x - y for x, y in zip(other_pos, self.pos[veh.sysid])])
            )
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

    def is_plan_done(self, sysid: int) -> bool:
        """
        Listen for a STATUSTEXT("DONE") message and respond with COMMAND_ACK.
        """
        msg = self.conns[sysid].recv_match(type="STATUSTEXT", blocking=False)
        if msg and msg.text == "DONE":
            self.conns[sysid].mav.command_ack_send(
                command=3000, result=mavlink2.MAV_RESULT_ACCEPTED
            )
            print(f"✅ Vehicle {sysid} terminated")
            return True
        return False


class GCS(Oracle):
    def __init__(self, sysids: List[int], name: str = "blue"):
        self.name = name
        super().__init__(sysids, name=f"GCS {name}", base_port=BasePort.GCS)
        self.paths = {sysid: [] for sysid in sysids}

    def save_pos(self):
        for sysid, pos in self.pos.items():
            self.paths[sysid].append(pos)
