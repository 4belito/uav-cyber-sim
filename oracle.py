import numpy as np
from plan.core import ActionNames
from helpers.change_coordinates import local2global, global2local
from vehicle_logic import VehicleLogic, Neighbors
from collections import OrderedDict
from numpy.typing import NDArray
from typing import List, Set


class Oracle:
    """
    Oracle class for veh-veh communication and general simulation controls
    """

    def __init__(self, vehicles: List[VehicleLogic]) -> None:
        self.vehs = vehicles
        self.pos = dict.fromkeys(self.vehs)

    def remove(self, veh: VehicleLogic):
        """
        Remove vehicles from the enviroment
        """
        self.vehs.remove(veh)

    def gather_broadcasts(self):
        self.pos.update({veh: self.get_global_pos(veh).copy() for veh in self.vehs})

    def get_global_pos(self, veh: VehicleLogic):
        return local2global(veh.pos, veh.home)

    def update_neighbors(self, veh: VehicleLogic):
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


class GCS(Oracle):
    def __init__(self, vehicles: VehicleLogic):
        super().__init__(vehicles)
        self.paths = {v: [] for v in vehicles}

    def save_pos(self):
        for veh, pos in self.pos.items():
            if veh.is_onair():
                self.paths[veh].append(pos)
