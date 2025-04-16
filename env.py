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

    def get_closest_position(self, veh):
        if not veh.onair:
            return None
        other_vehs = self.vehs - {veh}
        if not other_vehs:
            return None
        other_pos = [
            local2global(other.curr_pos, other.home, pairwise=True)
            for other in other_vehs
            if other.curr_pos is not None
        ]
        if not other_pos:
            return None

        pos = local2global(veh.curr_pos, veh.home, pairwise=True)
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

    def get_position(self, veh: VehicleLogic):
        veh.set_local_position()

    def get_positions(self):
        for veh in self.vehs:
            if veh.onair:
                self.get_position(veh)

    # def send_neighborg_position(self, veh: VehicleLogic, neighborg_id: int):
    #     wp = veh.goal_wp

    #     print(f" pos {veh.global_pos}")
    #     print(f" obj {self.poss[neighborg_id-1]}")
    #     print(f"goal wp {veh.goal_wp}")
    #     local_obj_pos = global2local(
    #         self.poss[neighborg_id - 1], veh.home, pairwise=True
    #     )
    #     next_wp = Enviroment.get_avoidance_wp(
    #         pos=veh.curr_pos, obj_pos=local_obj_pos, goal_pos=wp
    #     )
    #     next_step = make_go_to(
    #         wp=next_wp, wp_margin=0.5, verbose=2, cause_text="(avoidance)"
    #     )
    #     next_step.bind_connection(veh.conn)
    #     veh.plan.current.add_now(next_step)
