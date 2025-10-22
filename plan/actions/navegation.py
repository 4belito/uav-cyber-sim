"""
Defines logic for navigating to local NED waypoints using MAVLink.

Includes:
- Functions to command and check UAV movement in local coordinates.
- Construction of Step and Action objects to integrate into mission plans.
"""

import logging

from pymavlink import mavutil

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import Frame, MsgID
from helpers.connections.mavlink.streams import (
    ask_msg,
    stop_msg,
)
from helpers.coordinates import ENU, ENUs
from plan.core import Action, ActionNames, Step

# class GoToLocal(Step):
#     """Step to move the UAV to a waypoint in local coordinates."""

#     def __init__(
#         self,
#         name: str,
#         wp: ENU,  # Local waypoint relative to home position
#         wp_margin: float = 0.5,
#         stop_asking_pos: bool = True,
#         msg_pos_interval: int = 100_000,
#     ):
#         super().__init__(name)
#         self.wp = wp
#         self.wp_margin = wp_margin
#         self.stop_asking_pos = stop_asking_pos
#         self.type_mask = int(0b110111111000)
#         self.msg_pos_interval = msg_pos_interval  # in microseconds

#     def exec_fn(self, conn: MAVConnection) -> None:
#         """Send a MAVLink command to move the UAV to a local waypoint."""
#         go_msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
#             10,
#             conn.target_system,
#             conn.target_component,
#             Frame.LOCAL_NED,
#             self.type_mask,
#             *self.wp.to_ned(),
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#             0,
#         )
#         conn.mav.send(go_msg)
#         ask_msg(conn, MsgID.LOCAL_POSITION_NED, interval=self.msg_pos_interval)

#     def check_fn(self, conn: MAVConnection) -> bool:
#         """
#         Check if the UAV has reached the target altitude within an acceptable
#         margin.
#         """
#         pos = get_ENU_position(conn)
#         if pos is not None:
#             dist = math.dist(pos, self.wp)
#             logging.debug(
#                 f"📍 Vehicle {conn.target_system}: Distance to target: {dist:.2f} m"
#             )
#             reached = dist < self.wp_margin
#         else:
#             reached = False
#         if reached and self.stop_asking_pos:
#             stop_msg(conn, MsgID.LOCAL_POSITION_NED)
#         return reached


class GoTo(Step):
    """Step to move the UAV to a global waypoint."""

    def __init__(
        self,
        wp: ENU,  # Global waypoint
        name: str,
        wp_margin: float = 0.5,
        msg_pos_interval: int = 100_000,
        stop_asking_pos: bool = True,
    ):
        super().__init__(name)
        self.wp = wp
        self.wp_margin = wp_margin
        self.msg_pos_interval = msg_pos_interval
        self.stop_asking_pos = stop_asking_pos
        self.type_mask = int(0b110111111000)

    def exec_fn(self, conn: MAVConnection) -> None:
        """Send a MAVLink command to move the UAV to a global waypoint."""
        gra_wp = self.origin.to_abs(self.wp)
        go_msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            10,
            conn.target_system,
            conn.target_component,
            Frame.GLOBAL_INT,
            self.type_mask,
            *gra_wp.to_global_int_alt_in_meters(),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        conn.mav.send(go_msg)
        ask_msg(conn, MsgID.GLOBAL_POSITION_INT, interval=self.msg_pos_interval)

    def check_fn(self, conn: MAVConnection) -> bool:
        """
        Check if the UAV has reached the target altitude within an acceptable
        margin.
        """
        pos = self.origin.get_enu_position(conn)
        if pos is not None:
            self.curr_pos = pos
            dist = ENU.distance(pos, self.wp)
            logging.debug(
                f"📍 Vehicle {conn.target_system}: Distance to target: {dist:.2f} m"
            )
            reached = dist < self.wp_margin
        else:
            reached = False
        if reached and self.stop_asking_pos:
            stop_msg(conn, MsgID.GLOBAL_POSITION_INT)
        return reached


## Make the action
# def make_path_local(wps: ENUs, wp_margin: float = 0.5) -> Action[Step]:
#     """Create a FLY action composed of multiple go-to waypoint steps."""
#     go_local_action = Action[Step](name=ActionNames.AVOIDANCE)
#     for wp in wps:
#         goto_step = GoToLocal(name=f"go to {wp.short()}", wp=wp, wp_margin=wp_margin)
#         go_local_action.add(goto_step)
#     return go_local_action


## Make the action
def make_path(wps: ENUs | None = None, wp_margin: float = 0.5) -> Action[Step]:
    """Create a FLY action composed of multiple go-to waypoint steps."""
    name = ActionNames.FLY
    go_global_action = Action[Step](name=name, emoji=name.emoji)
    if wps is None:
        return go_global_action  # Return empty action if no waypoints
    for wp in wps:
        goto_step = GoTo(name=f"go to {wp.short()}", wp=wp, wp_margin=wp_margin)
        go_global_action.add(goto_step)
    return go_global_action


# def exec_go_local(conn: MAVConnection, wp: ENU, ask_pos_interval: int = 100_000):
#     """Send a MAVLink command to move the UAV to a local waypoint."""
#     go_msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
#         10,
#         conn.target_system,
#         conn.target_component,
#         Frame.LOCAL_NED,
#         TYPE_MASK,
#         *wp.to_ned(),
#         0,
#         0,
#         0,
#         0,
#         0,
#         0,
#         0,
#         0,
#     )
#     conn.mav.send(go_msg)
#     ask_msg(conn, MsgID.LOCAL_POSITION_NED, interval=ask_pos_interval)


# def exec_go_global(conn: MAVConnection, wp: GRA, ask_pos_interval: int = 100_000):
#     """Send a MAVLink command to move the UAV to a local waypoint."""
#     go_msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(
#         10,
#         conn.target_system,
#         conn.target_component,
#         Frame.GLOBAL_INT,
#         TYPE_MASK,
#         *wp.to_global_int_alt_in_meters(),
#         0,
#         0,
#         0,
#         0,
#         0,
#         0,
#         0,
#         0,
#     )
#     conn.mav.send(go_msg)
#     ask_msg(conn, MsgID.GLOBAL_POSITION_INT, interval=ask_pos_interval)


# def check_reach_local(
#     conn: MAVConnection,
#     wp: ENU = ENU(0, 0, 10),
#     wp_margin: float = 0.5,
#     stop_asking: bool = True,
# ):
#     """Check if the UAV has reached the target altitude within an acceptable margin."""
#     pos = get_ENU_position(conn)
#     if pos is not None:
#         dist = math.dist(pos, wp)
#         logging.debug(
#             f"📍 Vehicle {conn.target_system}: Distance to target: {dist:.2f} m"
#         )
#         answer = dist < wp_margin
#     else:
#         answer = False
#     if answer and stop_asking:
#         stop_msg(conn, MsgID.LOCAL_POSITION_NED)
#     return answer, pos


# def check_reach_global(
#     conn: MAVConnection,
#     wp: GRA,
#     wp_margin: float = 0.5,
#     stop_asking: bool = True,
# ):
#     """Check if the UAV has reached the target altitude within an acceptable margin."""
#     pos = get_GRA_position(conn)
#     if pos is not None:
#         dist = GRA.distance(pos, wp)
#         logging.debug(
#             f"📍 Vehicle {conn.target_system}: Distance to target: {dist:.2f} m"
#         )
#         answer = dist < wp_margin
#     else:
#         answer = False
#     if answer and stop_asking:
#         stop_msg(conn, MsgID.GLOBAL_POSITION_INT)
#     return answer, pos
