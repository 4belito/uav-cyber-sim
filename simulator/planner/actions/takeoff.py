"""
Defines a TAKEOFF action using MAVLink commands.

Includes:
- TakeOff: copter takeoff via MAV_CMD_NAV_TAKEOFF in GUIDED mode.
- PlaneTakeOff: plane takeoff via PlaneMode.TAKEOFF then switch to GUIDED.
- make_takeoff: factory dispatching by firmware.
"""

import logging

from simulator.config import Firmware
from simulator.helpers.connections.mavlink.enums import (
    CmdNav,
    LandState,
    ModeFlag,
    MsgID,
    ParamType,
    PlaneMode,
)
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.planner.action import Action
from simulator.planner.step import Step


class TakeOff(Step):
    """Step to command the vehicle to take off to a specified altitude."""

    def __init__(
        self,
        name: str,
        altitude: float,
        ask_position: bool = True,
        stop_msg_position: bool = False,
    ) -> None:
        super().__init__(name=name)
        self._altitude = altitude
        self._ask_position = ask_position
        self._stop_msg_position = stop_msg_position

    def exec_fn(self) -> None:
        """Send TAKEOFF command to reach target altitude."""
        msg = self.conn.mav.command_long_encode(
            self.conn.target_system,
            self.conn.target_component,
            CmdNav.TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            self._altitude,
        )
        self.mav_manager.send(msg)
        msg = ask_msg(self.conn, MsgID.EXTENDED_SYS_STATE)
        self.mav_manager.send(msg)
        if self._ask_position:
            msg = ask_msg(self.conn, MsgID.GLOBAL_POSITION_INT)
            self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Check if vehicle is in TAKEOFF state."""
        msg = self.mav_manager.state.get("EXTENDED_SYS_STATE")
        take_off = bool(msg and msg.landed_state == LandState.IN_AIR)
        pos = self.get_enu_position()
        if pos is not None:
            self.current_pos = pos
            logging.info(f"Vehicle {self.sysid}: 📍 Position: {pos.short()}")
        if take_off:
            msg = stop_msg(self.conn, MsgID.EXTENDED_SYS_STATE)
            self.mav_manager.send(msg)
        if self._ask_position and self._stop_msg_position:
            msg = stop_msg(self.conn, MsgID.GLOBAL_POSITION_INT)
            self.mav_manager.send(msg)
        return take_off


class PlaneTakeOff(Step):
    """ArduPlane takeoff via TAKEOFF flight mode (mode 13), then switch to GUIDED.

    PlaneMode.TAKEOFF executes a proper runway roll and climb without any
    mission upload. Once the plane reaches IN_AIR state, we switch back to
    GUIDED for waypoint navigation via DO_REPOSITION.

    If land_bearing is provided, AUTOLAND_DIR_OFF is set before the mode switch so
    AUTOLAND approaches along that world-compass bearing (home_heading-relative offset
    computed at exec time from the SITL spawn heading threaded via bind()).
    """

    def __init__(
        self, name: str, altitude: float, land_bearing: float | None = None
    ) -> None:
        super().__init__(name=name)
        self._altitude = altitude
        self._land_bearing = land_bearing
        self._switched_to_guided: bool = False

    def exec_fn(self) -> None:
        """Set TKOFF_ALT (and AUTOLAND_DIR_OFF if needed), then switch to TAKEOFF mode."""
        if self._land_bearing is not None:
            autoland_param = (self._land_bearing - self.home_heading) % 360
            self.mav_manager.send(
                self.conn.mav.param_set_encode(
                    self.conn.target_system,
                    self.conn.target_component,
                    b"AUTOLAND_DIR_OFF",
                    float(autoland_param),
                    ParamType.REAL32,
                )
            )
        self.mav_manager.send(
            self.conn.mav.param_set_encode(
                self.conn.target_system,
                self.conn.target_component,
                b"TKOFF_ALT",
                float(self._altitude),
                ParamType.INT16,
            )
        )
        self.mav_manager.send(
            self.conn.mav.set_mode_encode(
                self.conn.target_system,
                ModeFlag.CUSTOM_MODE_ENABLED,
                PlaneMode.TAKEOFF.value,
            )
        )
        self.mav_manager.send(ask_msg(self.conn, MsgID.EXTENDED_SYS_STATE))
        self.mav_manager.send(ask_msg(self.conn, MsgID.GLOBAL_POSITION_INT))

    def check_fn(self) -> bool:
        """Wait for IN_AIR, then switch back to GUIDED."""
        msg = self.mav_manager.state.get("EXTENDED_SYS_STATE")
        in_air = bool(msg and msg.landed_state == LandState.IN_AIR)
        pos = self.get_enu_position()
        if pos is not None:
            self.current_pos = pos
            logging.info(f"Vehicle {self.sysid}: 📍 Position: {pos.short()}")
        if in_air and not self._switched_to_guided:
            self.mav_manager.send(
                self.conn.mav.set_mode_encode(
                    self.conn.target_system,
                    ModeFlag.CUSTOM_MODE_ENABLED,
                    PlaneMode.GUIDED.value,
                )
            )
            self._switched_to_guided = True
        return self._switched_to_guided


def make_takeoff(
    altitude: float = 1.0,
    firmware: Firmware = "ArduCopter",
    land_bearing: float | None = None,
) -> Action[Step]:
    """Create a TAKEOFF action dispatched by firmware."""
    name = Action.Names.TAKEOFF
    action = Action[Step](name=name, emoji=name.emoji)
    if firmware == "ArduPlane":
        action.add(
            PlaneTakeOff(
                name=f"take off to {altitude} m",
                altitude=altitude,
                land_bearing=land_bearing,
            )
        )
    else:
        action.add(TakeOff(name=f"take off to {altitude} m", altitude=altitude))
    return action
