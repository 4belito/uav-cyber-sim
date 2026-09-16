"""Defines a LAND action with execution and landing check using MAVLink commands."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from simulator.helpers.connections.mavlink.enums import (
    Cmd,
    CmdNav,
    Frame,
    LandState,
    ModeFlag,
    MsgID,
    ParamType,
    PlaneMode,
)
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.helpers.coordinates import ENU, ENUPose
from simulator.planner.action import Action
from simulator.planner.step import Step

if TYPE_CHECKING:
    from simulator.config import Firmware


class Land(Step):
    """Step to land the vehicle."""

    def __init__(
        self,
        name: str,
        msg_land_interval: int,
        msg_pos_interval: int,
        stop_asking_pos: bool = True,
    ) -> None:
        super().__init__(name)
        self.msg_land_interval = msg_land_interval
        self.msg_pos_interval = msg_pos_interval
        self.stop_asking_pos = stop_asking_pos

    def exec_fn(self) -> None:
        """Send a MAVLink command to initiate landing."""
        msg = self.conn.mav.command_long_encode(
            self.conn.target_system,
            self.conn.target_component,
            CmdNav.LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.mav_manager.send(msg)
        msg = ask_msg(
            self.conn, MsgID.EXTENDED_SYS_STATE, interval=self.msg_land_interval
        )
        self.mav_manager.send(msg)
        msg = ask_msg(
            self.conn, MsgID.GLOBAL_POSITION_INT, interval=self.msg_pos_interval
        )
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Check if the vehicle has landed using EXTENDED_SYS_STATE."""
        msg = self.mav_manager.state.get("EXTENDED_SYS_STATE")
        current_pos = self.get_enu_position()
        if current_pos is not None:
            self.current_pos = current_pos
            logging.debug(f"Vehicle {self.sysid}: Altitude: {current_pos[2]:.2f} m")
        on_ground = bool(msg and msg.landed_state == LandState.ON_GROUND)
        if on_ground:
            msg = stop_msg(self.conn, MsgID.EXTENDED_SYS_STATE)
            self.mav_manager.send(msg)
            if self.stop_asking_pos:
                msg = stop_msg(self.conn, MsgID.LOCAL_POSITION_NED)
                self.mav_manager.send(msg)
            logging.info(f"Vehicle {self.sysid}: 🛬 Landed successfully.")
        return on_ground


class PlaneLand(Step):
    """
    ArduPlane landing via AUTOLAND mode (mode 26).

    AUTOLAND uses the takeoff heading captured by PlaneMode.TAKEOFF to set up
    a glide-slope approach and land without any mission upload. Requires that
    the plane was taken off via PlaneMode.TAKEOFF so initial_direction is set.

    land_wp specifies the landing position and desired approach heading.
    The position (x, y) is applied by relocating HOME via MAV_CMD_DO_SET_HOME
    before switching to AUTOLAND — AUTOLAND always targets HOME, so this
    determines where the plane touches down. The heading is sent as the home
    yaw. Note: the traffic-pattern approach direction is determined by the
    initial_direction captured during TAKEOFF mode, not by the home yaw; the
    heading here is stored on the home point but does not override the
    glide-slope track (which is fixed at takeoff time).

    autoland_alt overrides AUTOLAND_WP_ALT when provided. Leave it None to
    use whatever the model's parm file sets (recommended — each model's parm
    file pairs AUTOLAND_WP_ALT with AUTOLAND_WP_DIST for the correct glide
    slope at that scale).
    """

    def __init__(
        self,
        name: str,
        msg_land_interval: int,
        msg_pos_interval: int,
        land_wp: ENUPose,
        autoland_alt: float | None = None,
        autoland_wp_dist: float | None = None,
    ) -> None:
        super().__init__(name)
        self.msg_land_interval = msg_land_interval
        self.msg_pos_interval = msg_pos_interval
        self.land_wp: ENUPose = land_wp
        self.autoland_alt = autoland_alt
        self.autoland_wp_dist = autoland_wp_dist

    def exec_fn(self) -> None:
        """
        Relocate HOME, optionally override AUTOLAND params, then switch to AUTOLAND
        mode.
        """
        gra_wp = self.origin.to_abs(self.land_wp)
        self.mav_manager.send(
            self.conn.mav.command_int_encode(
                self.conn.target_system,
                self.conn.target_component,
                Frame.GLOBAL_RELATIVE_ALT,
                Cmd.DO_SET_HOME,
                0,  # current: 0 = use provided coordinates
                0,
                0.0,  # param1: 0 = use provided location (not current position)
                0.0,
                0.0,
                float(self.land_wp.heading),
                *gra_wp.to_global_int_alt_in_meters(),
            )
        )
        if self.autoland_wp_dist is not None:
            self.mav_manager.send(
                self.conn.mav.param_set_encode(
                    self.conn.target_system,
                    self.conn.target_component,
                    b"AUTOLAND_WP_DIST",
                    float(self.autoland_wp_dist),
                    ParamType.INT16,
                )
            )
        if self.autoland_alt is not None:
            self.mav_manager.send(
                self.conn.mav.param_set_encode(
                    self.conn.target_system,
                    self.conn.target_component,
                    b"AUTOLAND_WP_ALT",
                    float(self.autoland_alt),
                    ParamType.INT16,
                )
            )
        self.mav_manager.send(
            self.conn.mav.set_mode_encode(
                self.conn.target_system,
                ModeFlag.CUSTOM_MODE_ENABLED,
                PlaneMode.AUTOLAND.value,
            )
        )
        self.mav_manager.send(
            ask_msg(
                self.conn, MsgID.EXTENDED_SYS_STATE, interval=self.msg_land_interval
            )
        )
        self.mav_manager.send(
            ask_msg(
                self.conn, MsgID.GLOBAL_POSITION_INT, interval=self.msg_pos_interval
            )
        )

    def check_fn(self) -> bool:
        """Check if the vehicle has landed using EXTENDED_SYS_STATE."""
        msg = self.mav_manager.state.get("EXTENDED_SYS_STATE")
        current_pos = self.get_enu_position()
        if current_pos is not None:
            self.current_pos = current_pos
            dist = ENU.distance(current_pos, self.land_wp.unpose())
            logging.info(
                f"🛬 Vehicle {self.sysid}: dist: {dist:.2f}m alt={current_pos.z:.1f}m"
            )
        on_ground = bool(msg and msg.landed_state == LandState.ON_GROUND)
        if on_ground:
            self.mav_manager.send(stop_msg(self.conn, MsgID.EXTENDED_SYS_STATE))
            self.mav_manager.send(stop_msg(self.conn, MsgID.GLOBAL_POSITION_INT))
            logging.info(f"Vehicle {self.sysid}: 🛬 Landed successfully.")
        return on_ground


def make_land(
    land_wp: ENUPose | None = None,
    firmware: Firmware = "ArduCopter",
    msg_land_interval: int = 100_000,
    msg_pos_interval: int = 100_000,
    stop_asking_pos: bool = True,
    autoland_alt: float | None = None,
    autoland_wp_dist: float | None = None,
) -> Action[Step]:
    """Create a LAND Action with execution and check steps."""
    name = Action.Names.LAND
    land = Action[Step](name=name, emoji=name.emoji)
    if firmware == "ArduPlane":
        if land_wp is None:
            raise ValueError("land_wp is required for ArduPlane")
        land.add(
            PlaneLand(
                name="Land vehicle",
                msg_land_interval=msg_land_interval,
                msg_pos_interval=msg_pos_interval,
                land_wp=land_wp,
                autoland_alt=autoland_alt,
                autoland_wp_dist=autoland_wp_dist,
            )
        )
    else:
        land.add(
            Land(
                name="Land vehicle",
                msg_land_interval=msg_land_interval,
                msg_pos_interval=msg_pos_interval,
                stop_asking_pos=stop_asking_pos,
            )
        )
    return land
