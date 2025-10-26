"""Defines a LAND action with execution and landing check using MAVLink commands."""

import logging

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import CmdNav, LandState, MsgID
from helpers.connections.mavlink.streams import ask_msg, stop_msg
from helpers.coordinates import ENU
from planner.action import Action
from planner.step import Step


class Land(Step):
    """Step to land the UAV."""

    def __init__(
        self,
        name: str,
        final_wp: ENU,
        msg_land_interval: int,
        msg_pos_interval: int,
        stop_asking_pos: bool = True,
    ) -> None:
        super().__init__(name)
        self.msg_land_interval = msg_land_interval
        self.msg_pos_interval = msg_pos_interval
        self.stop_asking_pos = stop_asking_pos
        self.final_wp = final_wp

    def exec_fn(self, conn: MAVConnection) -> None:
        """Send a MAVLink command to initiate landing."""
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
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
        ask_msg(conn, MsgID.EXTENDED_SYS_STATE, interval=self.msg_land_interval)
        ask_msg(conn, MsgID.GLOBAL_POSITION_INT, interval=self.msg_pos_interval)

    def check_fn(self, conn: MAVConnection) -> bool:
        """Check if the UAV has landed using EXTENDED_SYS_STATE."""
        # parameter 4 is confirmation(it may be increased)
        msg = conn.recv_match(type="EXTENDED_SYS_STATE")
        current_pos = self.origin.get_enu_position(conn)
        if current_pos is not None:
            self.current_pos = current_pos
            logging.debug(
                f"Vehicle {conn.target_system}: Altitude: {current_pos[2]:.2f} m"
            )
        on_ground = bool(msg and msg.landed_state == LandState.ON_GROUND)
        if on_ground:
            stop_msg(conn, MsgID.EXTENDED_SYS_STATE)
            if self.stop_asking_pos:
                stop_msg(conn, MsgID.LOCAL_POSITION_NED)
            logging.info(f"Vehicle {conn.target_system}: 🛬 Landed successfully.")
        return on_ground


def make_land(
    final_wp: ENU,
    msg_land_interval: int = 100_000,
    msg_pos_interval: int = 100_000,
    stop_asking_pos: bool = True,
) -> Action[Step]:
    """Create a LAND Action with execution and check steps."""
    name = Action.Names.LAND
    land = Action[Step](name=name, emoji=name.emoji)
    land_step = Land(
        name="Land UAV",
        final_wp=final_wp,
        msg_land_interval=msg_land_interval,
        msg_pos_interval=msg_pos_interval,
        stop_asking_pos=stop_asking_pos,
    )
    land.add(land_step)
    return land
