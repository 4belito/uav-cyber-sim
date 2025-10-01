"""Utility functions for MAVLink connections and messaging."""

import logging
from typing import Literal, cast

from pymavlink import mavutil

from mavlink.customtypes.connection import MAVConnection
from mavlink.enums.autopilot import Autopilot
from mavlink.enums.type import Type


def connect(
    device: str, source_system: int = 255, source_component: int = 0
) -> MAVConnection:
    """
    Wrap `mavlink_connection` with a type cast to `MAVConnection`
    to enable clean static typing.
    Pass source_system and source_component to ensure correct sysid assignment.
    """
    return cast(
        MAVConnection,
        mavutil.mavlink_connection(  # type: ignore[arg-type]
            device, source_system=source_system, source_component=source_component
        ),
    )


# taken from mavproxy
def send_heartbeat(conn: MAVConnection, sysid: int = 255) -> None:
    """Send a GCS heartbeat message to the UAV."""
    # Set the source system ID for this connection
    conn.mav.srcSystem = sysid
    conn.mav.heartbeat_send(Type.GCS, Autopilot.INVALID, 0, 0, 0)


def create_udp_conn(
    base_port: int,
    offset: int,
    mode: Literal["receiver", "sender"],
) -> MAVConnection:
    """Create a MAVLink-over-UDP connection."""
    port = base_port + offset
    if mode == "receiver":
        conn = connect(f"udp:127.0.0.1:{port}")  # listen for incoming
        conn.wait_heartbeat()
    else:  # mode == "sender"
        conn = connect(f"udpout:127.0.0.1:{port}")  # send-only
    return conn


def create_tcp_conn(
    base_port: int,
    offset: int,
    role: Literal["client", "server"] = "client",
    sysid: int = 255,
) -> MAVConnection:
    """Create and in or out connection and wait for geting the hearbeat in."""
    port = base_port + offset
    connection_string = f"tcp{'in' if role == 'server' else ''}:127.0.0.1:{port}"

    try:
        conn = connect(connection_string)
        send_heartbeat(conn, sysid)
        conn.wait_heartbeat()
        conn.target_system = sysid
        return conn
    except Exception as e:
        logging.error(f"Failed to create TCP connection on port {port}: {e}")
        # logging.error(f"TCP connection error traceback:\n{traceback.format_exc()}")
        raise
