"""
Defines logic for navigating to local NED waypoints using MAVLink.

Includes:
- Functions to command and check UAV movement in local coordinates.
- Construction of Step and Action objects to integrate into mission plans.
"""

import logging
import math

from pymavlink import mavutil

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import Frame, MsgID
from helpers.connections.mavlink.streams import (
    ask_msg,
    get_ENU_position,
    get_GRA_position,
    stop_msg,
)
from helpers.coordinates import ENU, GRA, ENUs, GRAs
from plan.core import Action, ActionNames, Step


# TODO: Change this class to work with enu relatie to a origin instead of uav home
class GoToLocal(Step):
    """Step to move the UAV to a local waypoint."""

    def __init__(
        self,
        wp: ENU,
        name: str = "",
        wp_margin: float = 0.5,
        cause_text: str = "",
        stop_asking: bool = True,
    ):
        super().__init__(name)
        if not name:
            self.name = f"go to {fmt(wp)}"
            if cause_text:
                self.name += f" ({cause_text})"
        self.wp = wp
        self.cause_text = cause_text
        self.wp_margin = wp_margin
        self.stop_asking = stop_asking

    def exec_fn(self, conn: MAVConnection) -> None:
        """Send a MAVLink command to move the UAV to a local waypoint."""
        exec_go_local(conn, self.wp)

    def check_fn(self, conn: MAVConnection) -> bool:
        """
        Check if the UAV has reached the target altitude within an acceptable
        margin.
        """
        reached, pos = check_reach_local(
            conn, self.wp, self.wp_margin, stop_asking=self.stop_asking
        )
        if pos is not None:
            self.current_pos = pos
        return reached


# TODO: Change this class to work with enu relatie to a origin instead of uav home
class GoToGlobal(Step):
    """Step to move the UAV to a global waypoint."""

    def __init__(
        self,
        wp: GRA,
        name: str = "",
        wp_margin: float = 0.5,
        cause_text: str = "",
        stop_asking: bool = True,
    ):
        super().__init__(name)
        if not name:
            self.name = f"go to {fmt(wp)}"
            if cause_text:
                self.name += f" ({cause_text})"
        self.wp = wp
        self.cause_text = cause_text
        self.wp_margin = wp_margin
        self.stop_asking = stop_asking

    def exec_fn(self, conn: MAVConnection) -> None:
        """Send a MAVLink command to move the UAV to a global waypoint."""
        exec_go_global(conn, self.wp)

    def check_fn(self, conn: MAVConnection) -> bool:
        """
        Check if the UAV has reached the target altitude within an acceptable
        margin.
        """
        reached, pos = check_reach_global(
            conn, self.wp, self.wp_margin, stop_asking=self.stop_asking
        )
        if pos is not None:
            self.current_pos = pos
        return reached


## Make the action
def make_path_local(wps: ENUs | None = None, wp_margin: float = 0.5) -> Action[Step]:
    """Create a FLY action composed of multiple go-to waypoint steps."""
    go_local_action = Action[Step](name=ActionNames.AVOIDANCE, emoji="🛩️")
    if wps is None:
        return go_local_action  # Return empty action if no waypoints
    for wp in wps:
        goto_step = GoToLocal(wp=wp, wp_margin=wp_margin)
        go_local_action.add(goto_step)

    return go_local_action


## Make the action
def make_path_global(wps: GRAs | None = None, wp_margin: float = 0.5) -> Action[Step]:
    """Create a FLY action composed of multiple go-to waypoint steps."""
    go_global_action = Action[Step](name=ActionNames.AVOIDANCE, emoji="🛩️")
    if wps is None:
        return go_global_action  # Return empty action if no waypoints
    for wp in wps:
        goto_step = GoToGlobal(wp=wp, wp_margin=wp_margin)
        go_global_action.add(goto_step)

    return go_global_action


TYPE_MASK = int(0b110111111000)


def exec_go_local(conn: MAVConnection, wp: ENU, ask_pos_interval: int = 100_000):
    """Send a MAVLink command to move the UAV to a local waypoint."""
    go_msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,
        conn.target_system,
        conn.target_component,
        Frame.LOCAL_NED,
        TYPE_MASK,
        *wp.to_ned(),
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
    ask_msg(conn, MsgID.LOCAL_POSITION_NED, interval=ask_pos_interval)


def exec_go_global(conn: MAVConnection, wp: GRA, ask_pos_interval: int = 100_000):
    """Send a MAVLink command to move the UAV to a local waypoint."""
    go_msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        10,
        conn.target_system,
        conn.target_component,
        Frame.GLOBAL_INT,
        TYPE_MASK,
        *wp.to_global_int_alt_in_meters(),
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
    ask_msg(conn, MsgID.GLOBAL_POSITION_INT, interval=ask_pos_interval)


def check_reach_local(
    conn: MAVConnection,
    wp: ENU = ENU(0, 0, 10),
    wp_margin: float = 0.5,
    stop_asking: bool = True,
):
    """Check if the UAV has reached the target altitude within an acceptable margin."""
    pos = get_ENU_position(conn)
    if pos is not None:
        dist = math.dist(pos, wp)
        logging.debug(
            f"📍 Vehicle {conn.target_system}: Distance to target: {dist:.2f} m"
        )
        answer = dist < wp_margin
    else:
        answer = False
    if answer and stop_asking:
        stop_msg(conn, MsgID.LOCAL_POSITION_NED)
    return answer, pos


def check_reach_global(
    conn: MAVConnection,
    wp: GRA,
    wp_margin: float = 0.5,
    stop_asking: bool = True,
):
    """Check if the UAV has reached the target altitude within an acceptable margin."""
    pos = get_GRA_position(conn)
    if pos is not None:
        dist = GRA.distance(pos, wp)
        logging.debug(
            f"📍 Vehicle {conn.target_system}: Distance to target: {dist:.2f} m"
        )
        answer = dist < wp_margin
    else:
        answer = False
    if answer and stop_asking:
        stop_msg(conn, MsgID.GLOBAL_POSITION_INT)
    return answer, pos


def fmt(wp: ENU | GRA):
    """Format a waypoint as a tuple of readable values."""
    if isinstance(wp, ENU):
        return (int(x) if float(x).is_integer() else round(float(x), 2) for x in wp)
    return (
        round(wp.lat, 6),
        round(wp.lon, 6),
        int(wp.alt) if wp.alt.is_integer() else round(wp.alt, 2),
    )
