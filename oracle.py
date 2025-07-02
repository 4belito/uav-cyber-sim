"""
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
Define the Oracle class to simulate UAV-to-UAV communication.
Currently provides basic global position tracking and mission completion detection.
"""

from pymavlink.dialects.v20 import common as mavlink2  # type: ignore

from helpers.change_coordinates import pose  # ,global2local
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import ENU
from mavlink.util import CustomCmd, get_ENU_position


class Oracle:
    """
    Oracle class for vehicle-to-vehicle communication and simulation coordination.

    Establishes and maintains MAVLink connections to UAV logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(
        self, conns: list[MAVConnection], homes: list[ENU], name: str = "Oracle ⚪"
    ) -> None:
        self.pos: dict[int, ENU] = {}
        self.conns = {conn.target_system: conn for conn in conns}
        self.homes = homes
        self.name = name

    def remove(self, sysid: int):
        """Remove vehicles from the environment."""
        del self.conns[sysid]

    def gather_broadcasts(self):
        """Collect and store broadcasts (global positions so far) from all vehicles."""
        for sysid in self.conns:
            pos = self.get_global_pos(sysid)
            if pos is not None:
                self.pos[sysid] = pos

    def get_global_pos(self, sysid: int) -> ENU | None:
        """Get the current global position of the specified vehicle."""
        pos = get_ENU_position(self.conns[sysid])
        if pos is not None:
            pos = pose(pos, self.homes[sysid - 1])
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
        """Listen for a STATUSTEXT("DONE") message and respond with COMMAND_ACK."""
        conn = self.conns[sysid]
        msg = conn.recv_match(type="STATUSTEXT", blocking=False)
        if msg and msg.text == "DONE":
            conn.mav.command_ack_send(
                command=CustomCmd.PLAN_DONE, result=mavlink2.MAV_RESULT_ACCEPTED
            )
            print(f"✅ Vehicle {sysid} completed its mission")
            return True
        return False
