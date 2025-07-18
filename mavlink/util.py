"""
MAVLink command and message type definitions.

Provides enums for MAVLink commands, flight modes, parameter types, and required sensor
flags. Also defines protocol interfaces for typed access to MAVLink messages and
connections.
"""

from enum import IntEnum
from pathlib import Path
from typing import cast

import pymavlink.dialects.v20.ardupilotmega as mavlink
from pymavlink import mavutil

from helpers.change_coordinates import NED_to_ENU
from mavlink.customtypes.connection import MAVConnection
from mavlink.customtypes.location import ENU, GRA, NED, GRAs
from mavlink.enums import CmdNav, CmdSet, DataStream, Frame, MsgID


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


def request_sensor_streams(
    conn: MAVConnection,
    stream_ids: list[DataStream],
    rate_hz: int = 5,
) -> None:
    """Request sensor messages from ArduPilot."""
    for stream_id in stream_ids:
        conn.mav.request_data_stream_send(
            target_system=conn.target_system,
            target_component=conn.target_component,
            req_stream_id=stream_id,
            req_message_rate=rate_hz,
            start_stop=1,
        )


def get_ENU_position(conn: MAVConnection) -> ENU | None:
    """Request and return the UAV's current local NED position."""
    ## Check this to make blocking optional parameter
    msg = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.001)
    if msg:
        return NED_to_ENU(NED(msg.x, msg.y, msg.z))
    return None


def get_GRA_position(
    msg: mavlink.MAVLink_global_position_int_message, sysid: int, verbose: int = 1
) -> GRA:
    """Request and return the UAV's current local NED position."""
    ## Check this to make blocking optional parameter
    # msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=False, timeout=0.001)
    # This does not work. I'am not sure why
    # msg = conn.recv_match(type="LOCAL_POSITION_NED")
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0
    if verbose > 1:
        print(
            f"Vehicle {sysid}: 📍 Position: "
            f"lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m"
        )
    return GRA(lat, lon, alt)


def save_mission(path: Path, poses: GRAs, delay: int = 0) -> None:
    """
    Save a .waypoints file from a sequence of GRAPose ppositions.
    The file will include:
    - Home (index 0, altitude = 0).
    - Takeoff (index 1, to the first pose with alt).
    - Mission waypoints (indices 2 to N).
    - Return to launch (last index, with alt = 0).
    """
    WP = CmdNav.WAYPOINT.value
    TAKEOFF = CmdNav.TAKEOFF.value
    LAND = CmdNav.LAND.value
    REL_ALT = Frame.GLOBAL_RELATIVE_ALT.value
    DELAY = CmdNav.DELAY.value
    takeoff_idx = 1
    with path.open("w") as f:
        f.write("QGC WPL 110\n")

        # Home location
        home = poses[0]
        f.write(
            f"0\t0\t{REL_ALT}\t{WP}\t0\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t0.0\t1\n"
        )
        # Dalay mission (mission item with delay=0 does not work)
        if delay:
            f.write(
                f"1\t0\t{REL_ALT}\t{DELAY}\t{delay}\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t0.0\t1\n"
            )
            takeoff_idx += 1

        f.write(
            f"{takeoff_idx}\t0\t{REL_ALT}\t{TAKEOFF}\t0\t0\t0\t0\t{home.lat:.7f}\t{home.lon:.7f}\t{home.alt:.1f}\t1\n"
        )

        # Mission waypoints
        for i, pose in enumerate(poses[1:], start=takeoff_idx + 1):
            f.write(
                f"{i}\t0\t{REL_ALT}\t{WP}\t0\t0\t0\t0\t{pose.lat:.7f}\t{pose.lon:.7f}\t{pose.alt:.1f}\t1\n"
            )

        # Return to Launch (RTL)
        last = poses[-1]
        rtl_index = len(poses) + takeoff_idx
        f.write(
            f"{rtl_index}\t0\t{REL_ALT}\t{LAND}\t0\t0\t0\t0\t{last.lat:.7f}\t{last.lon:.7f}\t0.0\t1\n"
        )
