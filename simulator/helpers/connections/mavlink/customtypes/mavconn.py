"""
Protocols and type definitions for MAVLink message.

This module defines Protocols for various MAVLink messages and a typed MAVLink
connection interface.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal, Protocol, overload

if TYPE_CHECKING:
    import pymavlink.dialects.v20.ardupilotmega as mavlink


class MAVConnection(Protocol):
    """
    Protocol defining a typed MAVLink connection with support for recv_match
    and set_mode.
    """

    target_system: int
    target_component: int
    mav: mavlink.MAVLink

    @overload
    def recv_match(
        self,
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["STATUSTEXT"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_statustext_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["GLOBAL_POSITION_INT"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_global_position_int_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["LOCAL_POSITION_NED"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_local_position_ned_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["COMMAND_ACK"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_command_ack_message | None: ...

    def recv_msg(self) -> mavlink.MAVLink_message | None:
        """Receive the next MAVLink message (non-blocking)."""

    def write(self, data: bytes) -> None:
        """Send raw MAVLink-encoded bytes through the connection."""

    def wait_heartbeat(self) -> None:
        """Block until a heartbeat is received."""

    def set_mode(self, mode: int) -> None:
        """Set the UAV flight mode."""

    def close(self) -> None:
        """Close the MAVLink connection."""

    def waypoint_clear_all_send(self) -> None:
        """Send a command to clear all mission items on the vehicle."""

    def mission_count_send(
        self,
        target_system: int,
        target_component: int,
        count: int,
        mission_type: int = 0,
    ) -> None:
        """Send the mission count message to the UAV."""

    def mission_request_list_send(
        self,
        target_system: int,
        target_component: int,
        mission_type: int = 0,
    ) -> None:
        """Send MISSION_REQUEST_LIST; ArduPilot replies with MISSION_COUNT."""

    def send(self, mavmsg: mavlink.MAVLink_message) -> None:
        """Send the mission item."""
