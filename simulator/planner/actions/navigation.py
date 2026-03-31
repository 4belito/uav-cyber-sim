"""
Defines logic for navigating to local NED waypoints using MAVLink.

Includes:
- Functions to command and check UAV movement in local coordinates.
- Construction of Step and Action objects to integrate into mission plans.
"""

import logging

from simulator.helpers.connections.mavlink.enums import Frame, MsgID
from simulator.helpers.connections.mavlink.streams import (
    ask_msg,
    stop_msg,
)
from simulator.helpers.coordinates import ENU, ENUs
from simulator.planner.action import Action
from simulator.planner.step import Step


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

    def exec_fn(self) -> None:
        """Send a MAVLink command to move the UAV to a global waypoint."""
        gra_wp = self.origin.to_abs(self.wp)
        go_msg = self.conn.mav.set_position_target_global_int_encode(
            10,
            self.conn.target_system,
            self.conn.target_component,
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
        self.conn.mav.send(go_msg)
        msg = ask_msg(
            self.conn, MsgID.GLOBAL_POSITION_INT, interval=self.msg_pos_interval
        )
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """
        Check if the UAV has reached the target altitude within an acceptable
        margin.
        """
        # pos = self.origin.get_enu_position(self.conn)
        pos = self.get_enu_position()
        if pos is not None:
            self.current_pos = pos
            dist = ENU.distance(pos, self.wp)
            logging.info(f"📍 Vehicle {self.sysid}: Distance to target: {dist:.2f}m")
            reached = dist < self.wp_margin
        else:
            reached = False
        if reached and self.stop_asking_pos:
            msg = stop_msg(self.conn, MsgID.GLOBAL_POSITION_INT)
            self.mav_manager.send(msg)
        return reached


## Make the action
def make_path(wps: ENUs | None = None, wp_margin: float = 0.5) -> Action[Step]:
    """Create a FLY action composed of multiple go-to waypoint steps."""
    name = Action.Names.FLY
    go_global_action = Action[Step](name=name, emoji=name.emoji)
    if wps is None:
        return go_global_action  # Return empty action if no waypoints
    for wp in wps:
        goto_step = GoTo(name=f"go to {wp.short()}", wp=wp, wp_margin=wp_margin)
        go_global_action.add(goto_step)
    return go_global_action
