"""
Define the UAVMonitor class to UAV monitoring.
Currently provides basic global position tracking and mission completion detection.
"""

import logging

import pymavlink.dialects.v20.ardupilotmega as mavlink
from pymavlink.dialects.v20 import common as mavlink2  # type: ignore

from helpers.change_coordinates import GRA  # ,global2local
from mavlink.customtypes.connection import MAVConnection
from mavlink.util import CustomCmd, get_GRA_position


class UAVMonitor:
    """
    UAVMonitor class for UAV monitoring.

    Establishes and maintains MAVLink connections to UAV logic processes, retrieves
    positions, and listens for plan-completion signals.
    """

    def __init__(self, conns: dict[int, MAVConnection]) -> None:
        self.pos: dict[int, GRA] = {}
        self.conns = conns

    def remove_uav(self, sysid: int):
        """Remove vehicles from the environment."""
        del self.conns[sysid]
        del self.pos[sysid]

    def gather_broadcasts(self):
        """Collect and store broadcasts (global positions so far) from all vehicles."""
        for sysid in self.conns:
            self.get_global_pos(sysid)

    def get_global_pos(self, sysid: int):
        """Get the current global position of the specified vehicle."""
        msg = self.conns[sysid].recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=0.001
        )
        if not msg:
            return None
        self._get_global_pos(
            msg,
            sysid,
        )

    def _get_global_pos(
        self, msg: mavlink.MAVLink_global_position_int_message, sysid: int
    ):
        """Get the current global position of the specified vehicle."""
        self.pos[sysid] = get_GRA_position(msg, sysid)

    def is_plan_done(self, sysid: int) -> bool:
        """Listen for a STATUSTEXT("DONE") message and respond with COMMAND_ACK."""
        conn = self.conns[sysid]
        msg = conn.recv_match(type="STATUSTEXT", blocking=False)
        return bool(msg and self._is_plan_done(conn, msg, sysid))

    def _is_plan_done(
        self, conn: MAVConnection, msg: mavlink.MAVLink_statustext_message, sysid: int
    ) -> bool:
        """Check for a STATUSTEXT("DONE") message and respond with COMMAND_ACK."""
        if msg.text == "DONE":
            conn.mav.command_ack_send(
                command=CustomCmd.PLAN_DONE, result=mavlink2.MAV_RESULT_ACCEPTED
            )
            logging.info(f"✅ Vehicle {sysid} completed its mission")
            return True
        return False
