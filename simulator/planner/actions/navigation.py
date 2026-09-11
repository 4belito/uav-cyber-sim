"""
Defines logic for navigating to local NED waypoints using MAVLink.

Includes:
- GoTo: copter navigation via SET_POSITION_TARGET_GLOBAL_INT.
- PlaneGoTo: plane navigation via DO_REPOSITION in GUIDED mode.
- make_path: factory dispatching by firmware.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from simulator.helpers.connections.mavlink.enums import Cmd, Frame, MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.helpers.coordinates import ENU, ENUs
from simulator.planner.action import Action
from simulator.planner.step import Step

if TYPE_CHECKING:
    from simulator.config import Firmware


class GoTo(Step):
    """Step to move the vehicle to a global waypoint."""

    def __init__(
        self,
        wp: ENU,
        name: str,
        wp_margin: float | None = None,
        msg_pos_interval: int = 100_000,
        stop_asking_pos: bool = True,
    ):
        super().__init__(name)
        self.wp = wp
        self.wp_margin = wp_margin if wp_margin is not None else 0.5
        self.msg_pos_interval = msg_pos_interval
        self.stop_asking_pos = stop_asking_pos
        self.type_mask = 0b110111111000

    def exec_fn(self) -> None:
        """Send SET_POSITION_TARGET_GLOBAL_INT to the target waypoint."""
        self.send_position_target(self.wp, self.type_mask)
        self.mav_manager.send(
            ask_msg(
                self.conn, MsgID.GLOBAL_POSITION_INT, interval=self.msg_pos_interval
            )
        )

    def check_fn(self) -> bool:
        """Check if the vehicle has reached the target within wp_margin."""
        pos = self.get_enu_position()
        if pos is not None:
            self.current_pos = pos
            dist = ENU.distance(pos, self.wp)
            logging.info(f"📍 Vehicle {self.sysid}: Distance to target: {dist:.2f}m")
            if dist < self.wp_margin:
                if self.stop_asking_pos:
                    self.mav_manager.send(
                        stop_msg(self.conn, MsgID.GLOBAL_POSITION_INT)
                    )
                return True
        return False


class PlaneGoTo(Step):
    """
    GoTo step for ArduPlane using DO_REPOSITION in GUIDED mode.

    SET_POSITION_TARGET_GLOBAL_INT is ignored for lat/lon by ArduPlane; it only
    processes the altitude component. DO_REPOSITION calls set_guided_WP() with
    the full location and is the correct guidance command for fixed-wing GUIDED mode.

    Unlike a copter, a fixed-wing plane cannot stop at a point. When ArduPlane
    receives DO_REPOSITION it flies toward the target and then enters a loiter
    orbit around it at WP_LOITER_RAD (firmware default: 60 m). The plane
    therefore never gets closer than WP_LOITER_RAD to the waypoint center.
    Set WP_LOITER_RAD in your model's parm file to reflect the minimum turn
    radius the airframe can sustain at cruise speed: R_min = v²/(g·tan(bank)).

    wp_margin MUST exceed the model's WP_LOITER_RAD. The default of 70 m is
    conservative enough to work with the firmware default of 60 m on any model.
    For a model with a smaller WP_LOITER_RAD (e.g. 20 m for Zephyr), pass a
    tighter wp_margin (e.g. 25–30 m) to get more accurate waypoint tracking.
    """

    def __init__(
        self,
        wp: ENU,
        name: str,
        wp_margin: float | None = None,  # must be > WP_LOITER_RAD (60 m); default 70 m
        msg_pos_interval: int = 100_000,
        stop_asking_pos: bool = True,
    ):
        super().__init__(name)
        self.wp = wp
        self.wp_margin = wp_margin if wp_margin is not None else 70.0
        self.msg_pos_interval = msg_pos_interval
        self.stop_asking_pos = stop_asking_pos

    def exec_fn(self) -> None:
        """Send DO_REPOSITION to the target lat/lon/alt."""
        gra_wp = self.origin.to_abs(self.wp)
        msg = self.conn.mav.command_int_encode(
            self.conn.target_system,
            self.conn.target_component,
            Frame.GLOBAL_RELATIVE_ALT,
            Cmd.DO_REPOSITION,
            0,
            0,
            -1.0,  # param1: speed (-1 = no change)
            0.0,  # param2: bitmask
            0.0,  # param3: radius (0 = WP_LOITER_RAD)
            float("nan"),  # param4: yaw
            *gra_wp.to_global_int_alt_in_meters(),
        )
        self.mav_manager.send(msg)
        self.mav_manager.send(
            ask_msg(
                self.conn, MsgID.GLOBAL_POSITION_INT, interval=self.msg_pos_interval
            )
        )

    def check_fn(self) -> bool:
        """Check if the vehicle has reached within wp_margin of the target."""
        pos = self.get_enu_position()
        if pos is not None:
            self.current_pos = pos
            dist = ENU.distance(pos, self.wp)
            logging.info(f"📍 Vehicle {self.sysid}: Distance to target: {dist:.2f}m")
            if dist < self.wp_margin:
                if self.stop_asking_pos:
                    self.mav_manager.send(
                        stop_msg(self.conn, MsgID.GLOBAL_POSITION_INT)
                    )
                return True
        return False


def make_path(
    wps: ENUs | None = None,
    wp_margin: float | None = None,
    stop_asking_pos: bool = True,
    firmware: Firmware = "ArduCopter",
) -> Action[Step]:
    """Create a FLY action with one step per waypoint, dispatched by firmware."""
    name = Action.Names.FLY
    action = Action[Step](name=name, emoji=name.emoji)
    if wps is None:
        return action
    step_cls: type[GoTo] | type[PlaneGoTo] = (
        PlaneGoTo if firmware == "ArduPlane" else GoTo
    )
    for wp in wps:
        action.add(
            step_cls(
                name=f"go to {wp.short()}",
                wp=wp,
                wp_margin=wp_margin,
                stop_asking_pos=stop_asking_pos,
            )
        )
    return action
