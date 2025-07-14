"""
Protocols and type definitions for MAVLink message.

This module defines Protocols for various MAVLink messages and a typed MAVLink
connection interface.
"""

from typing import Literal, Protocol, overload

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
        type: Literal["HEARTBEAT"],
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_heartbeat_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["PARAM_VALUE"],
        timeout: float | None = ...,
    ) -> mavlink.MAVLink_param_value_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["EXTENDED_SYS_STATE"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_extended_sys_state_message | None: ...

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
        type: Literal["EKF_STATUS_REPORT"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_ekf_status_report_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["GPS_RAW_INT"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_gps_raw_int_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["SYS_STATUS"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_sys_status_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["COMMAND_ACK"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_command_ack_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["MISSION_ACK"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_mission_ack_message | None: ...

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
        type: Literal["MISSION_REQUEST"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_mission_request_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["MISSION_REQUEST_INT"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_mission_request_int_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["MISSION_ITEM_REACHED"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_mission_item_reached_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["MISSION_ITEM"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_mission_item_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["MISSION_COUNT"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_mission_count_message | None: ...

    @overload
    def recv_match(
        self,
        type: Literal["GLOBAL_POSITION_INT"],
        timeout: float | None = ...,
        blocking: bool | None = ...,
    ) -> mavlink.MAVLink_global_position_int_message | None: ...

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

    def send(self, mavmsg: mavlink.MAVLink_message) -> None:
        """Send the mission item."""
