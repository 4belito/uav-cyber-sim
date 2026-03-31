"""
Defines a TAKEOFF action using MAVLink commands.

Includes:
- exec_takeoff: sends a takeoff command to the UAV.
- check_takeoff: verifies if the UAV is currently taking off.
- make_takeoff: creates a takeoff Action with one execution step.
"""

import logging

from simulator.helpers.connections.mavlink.enums import CmdNav, LandState, MsgID
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.planner.action import Action
from simulator.planner.step import Step


class TakeOff(Step):
    """Step to command the UAV to take off to a specified altitude."""

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
        """Check if UAV is in TAKEOFF state."""
        msg = self.mav_manager.state.get("EXTENDED_SYS_STATE")
        take_off = bool(msg and msg.landed_state == LandState.IN_AIR)
        pos = self.get_enu_position()
        if pos is not None:
            self.current_pos = pos
            logging.info(
                f"Vehicle {self.conn.target_system}: 📍 Position: {pos.short()}"
            )
        if take_off:
            msg = stop_msg(self.conn, MsgID.EXTENDED_SYS_STATE)
            self.mav_manager.send(msg)
        if self._ask_position and self._stop_msg_position:
            msg = stop_msg(self.conn, MsgID.GLOBAL_POSITION_INT)
            self.mav_manager.send(msg)
        return take_off


def make_takeoff(altitude: float = 1.0) -> Action[Step]:
    """Create a TAKEOFF action with execution and check steps."""
    name = Action.Names.TAKEOFF
    takeoff_action = Action[Step](name=name, emoji=name.emoji)
    takeoff_action.add(TakeOff(name=f"take off to {altitude} m", altitude=altitude))
    return takeoff_action
