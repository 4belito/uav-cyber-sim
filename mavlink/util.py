"""
MAVLink command and message type definitions.

Provides enums for MAVLink commands, flight modes, parameter types, and required sensor
flags. Also defines protocol interfaces for typed access to MAVLink messages and
connections.
"""

from enum import IntEnum
from pathlib import Path
from typing import cast

from pymavlink import mavutil

from helpers.change_coordinates import NED_to_ENU
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import ENU, NED, GRAs
from mavlink.enums import CmdNav, CmdSet, Frame, MsgID


def connect(device: str) -> MAVConnection:
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
    if verbose > 2:
        print(
            f"Vehicle {conn.target_system}: 📡 Requested message "
            f"{MsgID(msg_id).name} at {1e6 / interval:.2f} Hz"
        )


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


def get_ENU_position(conn: MAVConnection) -> ENU | None:
    """Request and return the UAV's current local NED position."""
    ## Check this to make blocking optional parameter
    msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.001)
    # This does not work. I'am not sure why
    # msg = conn.recv_match(type="LOCAL_POSITION_NED")
    if msg:
        return NED_to_ENU(NED(msg.x, msg.y, msg.z))
    return None


def save_mission(name: str, poses: GRAs) -> None:
    """
    Save a .waypoints file from a sequence of GRAPose ppositions.
    The file will include:
    - Home (index 0, altitude = 0).
    - Takeoff (index 1, to the first pose with alt).
    - Mission waypoints (indices 2 to N).
    - Return to launch (last index, with alt = 0).
    """
    path = Path(f"plan/missions/{name}.waypoints")
    wp = CmdNav.WAYPOINT.value
    takeoff = CmdNav.TAKEOFF.value
    land = CmdNav.LAND.value
    frame = Frame.GLOBAL_RELATIVE_ALT.value
    with path.open("w") as f:
        f.write("QGC WPL 110\n")

        # Home location
        home = poses[0]
        f.write(
            f"0\t0\t{frame}\t{wp}\t0\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t0.0\t1\n"
        )

        # Takeoff at first pose altitude (TPDO find out what parameter is 15)
        f.write(
            f"1\t0\t{frame}\t{takeoff}\t0\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t{home.alt:.1f}\t1\n"
        )

        # Mission waypoints
        for i, pose in enumerate(poses[1:], start=2):
            f.write(
                f"{i}\t0\t{frame}\t{wp}\t0\t0\t0\t0\t{pose.lat:.7f}\t{pose.lon:.7f}\t{pose.alt:.1f}\t1\n"
            )

        # Return to Launch (RTL)
        last = poses[-1]
        rtl_index = len(poses) + 1
        f.write(
            f"{rtl_index}\t0\t{frame}\t{land}\t0\t0\t0\t0\t{last.lat:.7f}\t{last.lon:.7f}\t0.0\t1\n"
        )
