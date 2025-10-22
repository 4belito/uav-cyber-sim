"""
Defines a TAKEOFF action using MAVLink commands.

Includes:
- exec_takeoff: sends a takeoff command to the UAV.
- check_takeoff: verifies if the UAV is currently taking off.
- make_takeoff: creates a takeoff Action with one execution step.
"""

import logging

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import CmdNav, LandState, MsgID
from helpers.connections.mavlink.streams import ask_msg, stop_msg
from plan.core import Action, ActionNames, Step


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

    def exec_fn(self, conn: MAVConnection) -> None:
        """Send TAKEOFF command to reach target altitude."""
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
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
        ask_msg(conn, MsgID.EXTENDED_SYS_STATE)
        if self._ask_position:
            ask_msg(conn, MsgID.GLOBAL_POSITION_INT)

    def check_fn(self, conn: MAVConnection) -> bool:
        """Check if UAV is in TAKEOFF state."""
        msg = conn.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=0.01)
        take_off = bool(msg and msg.landed_state == LandState.TAKEOFF)
        pos = self.origin.get_enu_position(conn)
        if pos is not None:
            self.current_pos = pos
            logging.info(f"Vehicle {conn.target_system}: 📍 Position: {pos.short()}")
        if take_off:
            stop_msg(conn, MsgID.EXTENDED_SYS_STATE)
        if self._ask_position and self._stop_msg_position:
            stop_msg(conn, MsgID.GLOBAL_POSITION_INT)
        return take_off


def make_takeoff(altitude: float = 1.0) -> Action[Step]:
    """Create a TAKEOFF action with execution and check steps."""
    name = ActionNames.TAKEOFF
    takeoff_action = Action[Step](name=name, emoji=name.emoji)
    takeoff_action.add(TakeOff(name=f"take off to {altitude} m", altitude=altitude))
    return takeoff_action
