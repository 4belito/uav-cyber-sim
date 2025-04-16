import numpy as np
from plan.core import ActionNames
from helpers.change_coordinates import local2global, global2local
from vehicle_logic import VehicleLogic, Neighbors

from numpy.typing import NDArray
from typing import List, Set


class Enviroment:
    """
    Enviroment class for veh-veh communication and general simulation controls
    """

    def __init__(self, vehicles: List[VehicleLogic]) -> None:
        self.vehs = set(vehicles)
        self.gather_broadcasts()

    def remove(self, veh: VehicleLogic):
        """
        Remove vehicles from the enviroment
        """
        self.vehs.remove(veh)

    def gather_broadcasts(self):
        self.veh_pos = {
            veh: local2global(veh.current_position().copy(), veh.home, pairwise=True)
            for veh in self.vehs
            if veh.is_onair()
        }

    def update_neighbors(self, veh):
        pos = local2global(veh.current_position(), veh.home, pairwise=True)

        neigh_vehs = []
        neigh_poss = []
        neigh_dists = []

        for other, other_pos in self.veh_pos.items():
            if other is veh:
                continue

            dist = np.linalg.norm(other_pos - pos)
            if dist < veh.radar_radius:
                neigh_vehs.append(other)
                neigh_poss.append(other_pos)  # this is a reference to the array
                neigh_dists.append(dist)

        # Perform transformation only on the selected ones
        if neigh_poss:
            neigh_poss = global2local(np.stack(neigh_poss), veh.home, pairwise=True)
            neigh_dists = (np.array(neigh_dists),)

        veh.neighbors = Neighbors(
            neigh_vehs,
            distances=neigh_dists,  # avoid np.stack if 1D
            positions=neigh_poss,
        )

    # def get_closest_position(self, veh: VehicleLogic):
    #     if not veh.is_onair():
    #         return None
    #     other_vehs = self.vehs - {veh}
    #     if not other_vehs:
    #         return None
    #     other_pos = [
    #         local2global(self.veh_pos.current_position(), other.home, pairwise=True)
    #         for other in other_vehs
    #         if other.is_onair()
    #     ]
    #     if not other_pos:
    #         return None

    #     pos = local2global(veh.current_position(), veh.home, pairwise=True)
    #     norms = np.linalg.norm(other_pos - pos, axis=1)
    #     min_index = np.argmin(norms)
    #     if norms[min_index] < veh.radar_radius:
    #         return other_pos[min_index]
    #     else:
    #         return None

    # def send_closest_pos(self, veh):
    #     pos = self.get_closest_position(veh)
    #     if pos is not None:
    #         veh.obst_pos = global2local(pos, veh.home, pairwise=True)
    #     else:
    #         veh.obst_pos = None


class GCS(Enviroment):
    def __init__(self, vehicles):
        super().__init__(vehicles)
        self.paths = {veh: [] for veh in self.vehs}

    def gather_broadcasts(self):
        self.veh_pos = {
            veh: local2global(veh.current_position().copy(), veh.home, pairwise=True)
            for veh in self.vehs
            if veh.is_onair()
        }

    def save_pos(self):
        for veh, pos in self.veh_pos.items():
            self.paths[veh].append(pos)
