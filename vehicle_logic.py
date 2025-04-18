from pymavlink import mavutil
import numpy as np

from plan.planner import Plan, State
from typing import List

# from helpers.change_coordinates import GLOBAL_switch_LOCAL_NED
from plan.actions import make_go_to


class VehicleMode:
    MISSION = "MISSION"  # or "MISSION", "WAYPOINT_NAV"
    AVOIDANCE = "AVOIDANCE"  # or "COLLISION_AVOIDANCE"
    DYNAMIC_MISSION = "DYNAMIC_MISSION"


class VehicleLogic:
    def __init__(
        self,
        sys_id: int,
        home: tuple,
        plan: Plan = None,
        safety_radius: float = 5,
        radar_radius: float = 4,
        verbose: int = 1,
    ):
        self.idx = sys_id
        self.conn = mavutil.mavlink_connection(f"udp:127.0.0.1:{14551+10*(sys_id-1)}")
        self.conn.wait_heartbeat()
        ## This positions are global
        self.home = np.array(home)
        self.verbose = verbose

        # Properties declaration
        self.mode = VehicleMode.MISSION
        self.plan = plan if plan is not None else Plan.basic()
        self.plan.bind(self.conn, verbose)
        # all these positions are local
        self.neighbors = Neighbors(vehicles=[])
        self.safety_radius: float = safety_radius
        self.radar_radius: float = radar_radius
        if verbose:
            print(f"Vehicle {self.idx} launched 🚀")

    def act(self):
        if self.neighbors.vehs:
            i = np.argmin(self.neighbors.dist)
            self.check_avoidance(obst_pos=self.neighbors.pos[i])
        self.plan.act()

    def inject_avoidance(self, avoid_pos: np.ndarray):
        avoid_step = make_go_to(wp=avoid_pos, wp_margin=0.5, cause_text="(avoidance)")
        avoid_step.bind_connection(self.conn)
        self.plan.current.add_now(avoid_step)

    def check_avoidance(self, obst_pos: np.ndarray):
        distance = np.linalg.norm(self.position - obst_pos)
        avoid_pos = self.get_avoidance_pos(obst_pos)
        if distance < self.safety_radius and avoid_pos is not None:
            if (
                self.mode == VehicleMode.AVOIDANCE
                and self.current_step.state == State.DONE
            ):
                self.inject_avoidance(avoid_pos)
            elif self.mode == VehicleMode.MISSION:
                self.inject_avoidance(avoid_pos)
                self.set_mode(VehicleMode.AVOIDANCE)
        else:
            if self.current_step.state == State.DONE:
                self.set_mode(VehicleMode.MISSION)

    def set_dynamic_mission(self, goal_wp: np.ndarray):
        self.set_mode(VehicleMode.DYNAMIC_MISSION)
        self.goal_wp = goal_wp

    def set_mode(self, new_mode: VehicleMode):
        if new_mode != self.mode:
            print(f"Vehigcle {self.idx} switched to mode: {new_mode}")
            self.mode = new_mode

    @property
    def current_action(self):
        return self.plan.current

    @property
    def current_step(self):
        return self.current_action.current

    @property
    def pos(self):
        return self.plan.curr_pos

    def is_onair(self):
        return self.plan.onair

    @property
    def target_pos(self):
        return self.current_step.target_pos

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
        curr_pos = self.pos.copy()
        obj_dir = (obst_pos - curr_pos)[:2]
        target_pos = self.target_pos
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
