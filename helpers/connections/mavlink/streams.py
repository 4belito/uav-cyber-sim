"""Helpers for MAVLink message streams."""

import logging

import pymavlink.dialects.v20.ardupilotmega as mavlink

from helpers.coordinates import ENU, GRA

from .customtypes.mavconn import MAVConnection
from .enums import CmdSet, DataStream, MsgID


def ask_msg(
    conn: MAVConnection,
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
    logging.debug(
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
        return ENU.from_ned(msg.x, msg.y, msg.z)
    return None


def get_GRA_position(
    msg: mavlink.MAVLink_global_position_int_message, sysid: int
) -> GRA:
    """Request and return the UAV's current local NED position."""
    gra = GRA.from_global_int(msg.lat, msg.lon, msg.relative_alt)
    logging.debug(f"Vehicle {sysid}: 📍 Position: {gra} m")
    return gra
