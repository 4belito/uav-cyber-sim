from pymavlink import mavutil
import numpy as np

from plan.planner import Plan, State
from plan.core import ActionNames
from typing import List

# from helpers.change_coordinates import GLOBAL_switch_LOCAL_NED
from plan.actions import get_local_position, make_go_to
from helpers import local2global, global2local


class VehicleMode:
    MISSION = "MISSION"  # or "MISSION", "WAYPOINT_NAV"
    AVOIDANCE = "AVOIDANCE"  # or "COLLISION_AVOIDANCE"
    DYNAMIC_MISSION = "DYNAMIC_MISSION"


class VehicleLogic:
    def __init__(
        self,
        sys_id: int,
        offset: tuple,
        plan: Plan = None,
        verbose: int = 1,
    ):
        self.sys_id = sys_id
        self.conn = mavutil.mavlink_connection(f"udp:127.0.0.1:{14551+10*(sys_id-1)}")
        self.conn.wait_heartbeat()
        ## This positions are global
        self.offset = offset
        self.home = np.array(offset[:3])
        self.verbose = verbose

        # Properties declaration
        self.mode = VehicleMode.MISSION
        self.plan = plan if plan is not None else Plan.basic()
        self.start_plan()
        # all these positions are local
        self.neighbors: Neighbors = None
        self.safety_radius: float = 5
        self.radar_radius: float = 10
        if verbose:
            print(f"Vehicle {self.sys_id} launched 🚀")

    def act(self):
        if self.neighbors.vehs:
            i = np.argmin(self.neighbors.dist)
            self.check_avoidance(obst_pos=self.neighbors.pos[i])
        self.plan.act()

    def avoidance_plan_update(self, avoid_pos):
        avoid_step = make_go_to(
            wp=avoid_pos, wp_margin=0.5, verbose=2, cause_text="(avoidance)"
        )
        avoid_step.bind_connection(self.conn)
        self.plan.current.add_now(avoid_step)

    def check_avoidance(self, obst_pos):
        distance = np.linalg.norm(self.current_position() - obst_pos)
        avoid_pos = self.get_avoidance_pos(obst_pos)
        if distance < self.safety_radius and avoid_pos is not None:
            if (
                self.mode == VehicleMode.AVOIDANCE
                and self.current_step().state == State.DONE
            ):
                self.avoidance_plan_update(avoid_pos)
            elif self.mode == VehicleMode.MISSION:
                self.avoidance_plan_update(avoid_pos)
                self.set_mode(VehicleMode.AVOIDANCE)
        else:
            if self.current_step().state == State.DONE:
                self.set_mode(VehicleMode.MISSION)

    def start_plan(self):
        self.plan.start(self.conn)

    def set_dynamic_mission(self, goal_wp: np.ndarray):
        self.set_mode(VehicleMode.DYNAMIC_MISSION)
        self.goal_wp = goal_wp

    def set_mode(self, new_mode: VehicleMode):
        if new_mode != self.mode:
            print(f"Vehigcle {self.sys_id} switched to mode: {new_mode}")
            self.mode = new_mode

    def get_mode(self):
        return self.mode

    def current_action(self):
        return self.plan.current

    def current_step(self):
        return self.current_action().current

    def current_position(self):
        return self.plan.curr_pos

    def is_onair(self):
        return self.plan.onair

    def target_position(self):
        return self.current_step().target_pos

    def get_avoidance_pos(
        self,
        obst_pos: np.ndarray,
        distance: float = 5,
        direction: str = "left",
    ):
        """
        Sends a velocity command in body frame, orthogonal to the direction of wp.
        `direction` can be 'left' or 'right' (relative to wp direction).
        """
        # Normalize wp direction (ignore Z)
        curr_pos = self.current_position().copy()
        obj_dir = (obst_pos - curr_pos)[:2]
        target_pos = self.target_position()
        target_dir = (target_pos - curr_pos)[:2]
        if np.dot(obj_dir, target_dir) < 0:
            return None
        obj_dir = obj_dir / np.linalg.norm(obj_dir) * distance
        # Get orthogonal direction
        if direction == "left":
            ortho = np.array([-obj_dir[1], obj_dir[0], 0])
        elif direction == "right":
            ortho = np.array([obj_dir[1], -obj_dir[0], 0])
        else:
            raise ValueError("Direction must be 'left' or 'right'")

        # Scale to desired speed
        return curr_pos + ortho


class Neighbors:
    def __init__(
        self,
        vehicles: List[VehicleLogic],
        distances: np.ndarray = None,
        positions: np.ndarray = None,
    ):
        self.vehs = vehicles
        self.dist = distances
        self.pos = positions
