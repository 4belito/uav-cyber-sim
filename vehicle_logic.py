# """Vehicle logic module for executing mission plans with mode switching."""

# from enum import StrEnum

# from mavlink.customtypes.connection import MAVConnection
# from mavlink.customtypes.location import ENU
# from plan.planner import Action, Plan, Step


# class VehicleMode(StrEnum):
#     """Defines operational modes for the UAV."""

#     MISSION = "MISSION"  # or "MISSION", "WAYPOINT_NAV"
#     AVOIDANCE = "AVOIDANCE"  # or "COLLISION_AVOIDANCE"


# # sis_id -> id and start at 0
# # sis_id may go out of the class
# class VehicleLogic:
#     """Handles the logic for executing a UAV's mission plan."""

#     def __init__(
#         self,
#         connection: MAVConnection,
#         plan: Plan,
#         safety_radius: float = 5,
#         radar_radius: float = 10,
#         verbose: int = 1,
#     ):
#         # Vehicle Creation
#         self.conn = connection
#         self.sysid = connection.target_system
#         self.verbose = verbose

#         # Mode Properties
#         self.mode = VehicleMode.MISSION
#         self.plan = plan
#         self.plan.bind(self.conn, verbose)
#         self.back_mode = VehicleMode.MISSION

#         # Communication properties (positions are local)
#         # self.neighbors = Neighbors(vehicles=[])
#         self.safety_radius: float = safety_radius
#         self.radar_radius: float = radar_radius

#     def act(self):
#         """Perform the next step in the mission plan."""
#         # if self.neighbors.vehs:
#         #     i = np.argmin(self.neighbors.dist)
#         #     self.check_avoidance(obst_pos=self.neighbors.pos[i])
#         # if self.plan.mode == PlanMode.DYNAMIC and self.mode != VehicleMode.AVOIDANCE:
#         #     self.check_dynamic_action()
#         self.plan.act()

#     def set_mode(self, new_mode: VehicleMode) -> None:
#         """Switch the vehicle to a new operational mode."""
#         if new_mode != self.mode:
#             print(f"Vehicle {self.sysid} switched to mode: 🔁 {new_mode}")
#             self.mode = new_mode

#     @property
#     def current_action(self) -> Action[Step] | None:
#         """Return the current action being executed."""
#         return self.plan.current

#     @property
#     def current_step(self) -> Step | None:
#         """Return the current step within the current action."""
#         if self.current_action is not None:
#             return self.current_action.current
#         else:
#             return None

#     @property
#     def pos(self) -> ENU | None:
#         """Return the current estimated position of the UAV."""
#         return self.plan.curr_pos

#     def is_onair(self) -> bool | None:
#         """Return whether the UAV is currently airborne."""
#         return self.plan.onair

#     @property
#     def target_pos(self) -> ENU | None:
#         """Return the current step's target position, if any."""
#         if self.current_step:
#             return self.current_step.target_pos
#         else:
#             return None

#     ## Passive avoidance
#     # def check_avoidance(self, obst_pos: np.ndarray):
#     #     obst_dist = np.linalg.norm(self.pos - obst_pos)
#     #     if obst_dist < self.safety_radius:
#     #         avoid_pos = self.get_avoidance_pos(obst_pos, obst_dist)
#     #         if avoid_pos is not None:
#     #             if (
#     #                 self.mode == VehicleMode.AVOIDANCE
#     #                 and self.current_step.state == State.DONE  # check this later
#     #             ):
#     #                 avoid_step = self.create_goto(
#     #                     pos=avoid_pos,
#     #                     target_pos=self.target_pos,
#     #                     cause_text="(avoidance)",
#     #                     is_improv=True,
#     #                 )
#     #                 self.inject(avoid_step)
#     #             elif self.mode != VehicleMode.AVOIDANCE:
#     #                 avoid_step = self.create_goto(
#     #                     pos=avoid_pos,
#     #                     target_pos=self.target_pos,
#     #                     cause_text="(avoidance)",
#     #                     is_improv=True,
#     #                 )
#     #                 self.inject(avoid_step)
#     #                 self.back_mode = self.mode
#     #                 self.set_mode(VehicleMode.AVOIDANCE)
#     #             return
#     #     if self.mode == VehicleMode.AVOIDANCE and
#     #                                   self.current_step.state == State.DONE:
#     #         self.set_mode(self.back_mode)

#     # def check_dynamic_action(self):
#     #     if (
#     #         self.current_action.name == ActionNames.FLY
#     #         and self.current_action.current.state == State.NOT_STARTED
#     #     ):
#     #         final_wp = self.plan.steps[-2].target_pos
#     #         dist = np.linalg.norm(self.pos - final_wp)
#     #         if dist > self.plan.wp_margin:
#     #             self.inject_dynamic_action()

#     # def inject_dynamic_action(self):
#     #     final_wp = self.plan.steps[-2].target_pos
#     #     valid_waypoints = get_valid_waypoints(
#     #         self.pos, final_wp, self.plan.dynamic_wps, eps=self.plan.wp_margin
#     #     )
#     #     if valid_waypoints.shape[0] == 0:
#     #         next_wp = adjust_one_significant_axis_toward_corridor(
#     #             self.pos, self.plan.dynamic_wps, eps=self.plan.wp_margin
#     #         )
#     #         goto_step = self.create_goto(
#     #             pos=next_wp, target_pos=next_wp, cause_text="(dynamic)",
#     #                                                                is_improv=True
#     #         )
#     #     else:
#     #         next_wp = find_best_waypoint(self.pos, final_wp, valid_waypoints)
#     #         goto_step = self.create_goto(
#     #             pos=next_wp, target_pos=next_wp, cause_text="(dynamic)",
#     #                                                                   is_improv=False
#     #         )
#     #     self.inject(goto_step)

#     # def inject(self, step):
#     #     if self.current_step.is_improv:
#     #         self.current_action.add_over(step)
#     #     else:
#     #         self.current_action.add_now(step)

#     # def create_goto(
#     #     self,
#     #     pos: np.ndarray,
#     #     target_pos: np.ndarray,
#     #     cause_text: str = "(avoidance)",
#     #     is_improv: bool = False,
#     # ):
#     #     goto_step = make_go_to(
#     #         wp=pos,
#     #         target_pos=target_pos,
#     #         wp_margin=self.plan.wp_margin,
#     #         cause_text=cause_text,
#     #         is_improv=is_improv,
#     #     )
#     #     goto_step.bind(self.conn)
#     #     return goto_step

#     # def get_avoidance_pos(
#     #     self,
#     #     obst_pos: np.ndarray,
#     #     obst_dist: np.ndarray,
#     #     direction: str = "left",
#     #     safety_eps: float = 0.0,
#     # ):
#     #     """
#     #     Sends a velocity command in body frame, orthogonal to the direction of wp.
#     #     `direction` can be 'left' or 'right' (relative to wp direction).
#     #     """
#     #     # Normalize wp direction (ignore Z)
#     #     curr_pos = self.pos.copy()
#     #     obj_dir = (obst_pos - curr_pos)[:2]
#     #     target_pos = self.target_pos
#     #     target_dir = (target_pos - curr_pos)[:2]
#     #     if np.dot(obj_dir, target_dir) < 0:
#     #         return None
#     #     distance = np.sqrt(self.safety_radius**2 - obst_dist**2) + safety_eps
#     #     obj_dir = obj_dir / np.linalg.norm(obj_dir) * distance
#     #     # Get orthogonal direction
#     #     if direction == "left":
#     #         ortho = np.array([-obj_dir[1], obj_dir[0], 0])
#     #     elif direction == "right":
#     #         ortho = np.array([obj_dir[1], -obj_dir[0], 0])
#     #     else:
#     #         raise ValueError("Direction must be 'left' or 'right'")

#     #     # Scale to desired speed
#     #     return curr_pos + ortho


# # class Neighbors:
# #     def __init__(
# #         self,
# #         vehicles: List[VehicleLogic],
# #         distances: np.ndarray | None = None,
# #         positions: np.ndarray | None = None,
# #     ):
# #         self.vehs = vehicles
# #         self.dist = distances
# #         self.pos = positions
