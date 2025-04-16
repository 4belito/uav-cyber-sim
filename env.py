import numpy as np
from plan.core import ActionNames
from helpers.change_coordinates import local2global, global2local
from vehicle_logic import VehicleLogic

from numpy.typing import NDArray
from typing import List, Set

air_actions = (ActionNames.TAKEOFF, ActionNames.FLY, ActionNames.LAND)


class Enviroment:
    """
    Enviroment class for veh-veh communication and general simulation controls
    """

    def __init__(self, vehicles: List[VehicleLogic]) -> None:
        self.vehs = set(vehicles)

    def remove(self, veh: VehicleLogic):
        """
        Remove vehicles from the enviroment
        """
        self.vehs.remove(veh)

    def get_closest_position(self, veh: VehicleLogic):
        if not veh.is_onair():
            return None
        other_vehs = self.vehs - {veh}
        if not other_vehs:
            return None
        other_pos = [
            local2global(other.current_position(), other.home, pairwise=True)
            for other in other_vehs
            if other.is_onair()
        ]
        if not other_pos:
            return None

        pos = local2global(veh.current_position(), veh.home, pairwise=True)
        norms = np.linalg.norm(other_pos - pos, axis=1)
        min_index = np.argmin(norms)
        if norms[min_index] < veh.radar_radius:
            return other_pos[min_index]
        else:
            return None

    def send_closest_pos(self, veh):
        pos = self.get_closest_position(veh)
        if pos is not None:
            veh.obst_pos = global2local(pos, veh.home, pairwise=True)
        else:
            veh.obst_pos = None
