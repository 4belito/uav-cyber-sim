from pymavlink import mavutil
import numpy as np

from plan.planner import Plan, State
from plan.core import ActionNames

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
        self.curr_pos = None
        self.goal_pos = None
        self.obst_pos = None

        self.onair = False
        self.safety_radius = 5
        self.radar_radius = 10
        if verbose:
            print(f"Vehicle {self.sys_id} launched 🚀")

    def act(self):
        if self.obst_pos is not None:
            self.check_avoidance()
        if self.mode == VehicleMode.AVOIDANCE:
            self.avoidance_plan_update()
        self.plan.act()
        self.update_oniar()

    def update_oniar(self):
        current_action = self.current_action()
        if current_action:
            if self.onair:
                self.onair = not (
                    current_action.name == ActionNames.LAND
                    and current_action.state == State.DONE
                )
            else:
                self.onair = (
                    current_action.name == ActionNames.TAKEOFF
                    and current_action.state == State.IN_PROGRESS
                )

    def avoidance_plan_update(self):
        avoid_pos = self.get_avoidance_pos()
        avoid_step = make_go_to(
            wp=avoid_pos, wp_margin=0.5, verbose=2, cause_text="(avoidance)"
        )
        avoid_step.bind_connection(self.conn)
        self.plan.current.add_now(avoid_step)

    def check_avoidance(self):
        distance = np.linalg.norm(self.curr_pos - self.obst_pos)
        if distance < self.safety_radius:
            self.set_mode(VehicleMode.AVOIDANCE)
        else:
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

    def reset_plan(self):
        pass

    def current_action(self):
        return self.plan.current

    def current_step(self):
        return self.current_action().current

    def set_local_position(self):
        answ = get_local_position(self.conn)
        if answ is not False:
            self.curr_pos = answ

    def get_global_position(self):
        self.set_local_position(self.conn)
        return local2global(self.curr_pos)

    def get_avoidance_pos(
        self,
        distance: float = 1,
        direction: str = "left",
    ):
        """
        Sends a velocity command in body frame, orthogonal to the direction of wp.
        `direction` can be 'left' or 'right' (relative to wp direction).
        """
        # Normalize wp direction (ignore Z)
        obj_dir = (self.obst_pos - self.curr_pos)[:2]
        goal_dir = (self.goal_pos - self.curr_pos)[:2]
        if np.dot(obj_dir, goal_dir) < 0:
            return self.goal_pos
        obj_dir = obj_dir / np.linalg.norm(obj_dir) * distance
        # Get orthogonal direction
        if direction == "left":
            ortho = np.array([-obj_dir[1], obj_dir[0], 0])
        elif direction == "right":
            ortho = np.array([obj_dir[1], -obj_dir[0], 0])
        else:
            raise ValueError("Direction must be 'left' or 'right'")

        # Scale to desired speed
        return self.curr_pos + ortho

    # def get_global_position(self):
    #     msg = self.conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    #     if msg:
    #         # Convert from scaled integers (E7 format) to decimal degrees
    #         lat = msg.lat / 1e7
    #         lon = msg.lon / 1e7
    #         alt = msg.alt / 1000  # Convert mm to meters
    #         return lat, lon, alt
    #     return None  # Return None if no data received

    # def move_left(self, speed=0.5):
    #     """
    #     Moves the UAV left by overriding position commands with a velocity command.

    #     - speed: Speed in m/s (default 0.5 m/s)
    #     """
    #     print(f"Moving left at {speed} m/s")
    #     # Step 1: Cancel Position Control (ignore position commands)
    #     type_mask = int(
    #         0b010111000111
    #     )  # Only use velocity, ignore position & acceleration
    #     coordinate_frame = mavutil.mavlink.MAV_FRAME_BODY_NED  # Relative to drone

    #     # Step 2: Send Velocity Command
    #     vy = -speed  # Negative vy moves left

    #     msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
    #         10,
    #         self.sys,
    #         self.comp,
    #         coordinate_frame,
    #         type_mask,
    #         0,
    #         0,
    #         0,  # Position ignored
    #         0,
    #         vy,
    #         0,  # Move left (velocity command)
    #         0,
    #         0,
    #         0,  # Acceleration ignored
    #         0,
    #         0,  # Ignore yaw
    #     )
    #     print("turn left msg will be sent")
    #     self.conn.mav.send(msg)
    #     print("turn left msgsent")
    #     # Wait for acknowledgment before continuing
    #     while not self.is_acknowledged():
    #         print(f"Waiting moving left acknowledge")
    #         pass  # Keep checking for an acknowledgment
