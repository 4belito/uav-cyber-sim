"""
MAVLink command and message type definitions.

Provides enums for MAVLink commands, flight modes, parameter types, and required sensor
flags. Also defines protocol interfaces for typed access to MAVLink messages and
connections.
"""

from enum import IntEnum
from typing import cast

from pymavlink import mavutil

from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import CmdSet, MsgID


def connection(device: str = "tcp:127.0.0.1:5760") -> MAVConnection:
    """
    Wrap `mavlink_connection` with a type cast to `MAVConnection`
    to enable clean static typing.
    """
    return cast(MAVConnection, mavutil.mavlink_connection(device))  # type: ignore[arg-type]


class CustomCmd(IntEnum):
    """official MAV_CMD values generally range from 0 to ~2999."""

    PLAN_DONE = 3000  # Custom command to mark end of plan


def ask_msg(
    conn: MAVConnection,
    verbose: int,
    msg_id: int,
    interval: int = 1_000_000,
) -> None:
    """Request periodic sending of a MAVLink message (1 Hz)."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        CmdSet.MESSAGE_INTERVAL,
        0,
        msg_id,
        interval,  # microseconds
        0,
        0,
        0,
        0,
        0,
    )
    if verbose == 2:
        print(f"📡 Requested message {MsgID(msg_id).name} at {1e6 / interval:.2f} Hz")


def stop_msg(conn: MAVConnection, msg_id: int) -> None:
    """Stop sending a specific MAVLink message."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        CmdSet.MESSAGE_INTERVAL,
        0,
        msg_id,
        -1,  # Stop
        0,
        0,
        0,
        0,
        0,
    )
