"""Typed interface for accessing MAVLink messages from VehicleState."""

from typing import Literal, Protocol, overload

import pymavlink.dialects.v20.ardupilotmega as mavlink


class VehicleStateP(Protocol):
    """Typed interface for accessing MAVLink messages from VehicleState."""

    # latest messages keyed by MAVLink message type name
    messages: dict[str, mavlink.MAVLink_message]

    # ---------------------------------------------------------
    # Typed accessors
    # ---------------------------------------------------------

    def update(self, msg: mavlink.MAVLink_message) -> None:
        """Update the vehicle state with a new MAVLink message."""

    @overload
    def get(
        self, msg_type: Literal["HEARTBEAT"]
    ) -> mavlink.MAVLink_heartbeat_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["PARAM_VALUE"]
    ) -> mavlink.MAVLink_param_value_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["EXTENDED_SYS_STATE"]
    ) -> mavlink.MAVLink_extended_sys_state_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["LOCAL_POSITION_NED"]
    ) -> mavlink.MAVLink_local_position_ned_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["EKF_STATUS_REPORT"]
    ) -> mavlink.MAVLink_ekf_status_report_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["GPS_RAW_INT"]
    ) -> mavlink.MAVLink_gps_raw_int_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["SYS_STATUS"]
    ) -> mavlink.MAVLink_sys_status_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["COMMAND_ACK"]
    ) -> mavlink.MAVLink_command_ack_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["MISSION_ACK"]
    ) -> mavlink.MAVLink_mission_ack_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["STATUSTEXT"]
    ) -> mavlink.MAVLink_statustext_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["MISSION_REQUEST"]
    ) -> mavlink.MAVLink_mission_request_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["MISSION_REQUEST_INT"]
    ) -> mavlink.MAVLink_mission_request_int_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["MISSION_ITEM_REACHED"]
    ) -> mavlink.MAVLink_mission_item_reached_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["MISSION_ITEM"]
    ) -> mavlink.MAVLink_mission_item_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["MISSION_COUNT"]
    ) -> mavlink.MAVLink_mission_count_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["MISSION_CURRENT"]
    ) -> mavlink.MAVLink_mission_current_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["GLOBAL_POSITION_INT"]
    ) -> mavlink.MAVLink_global_position_int_message | None: ...

    @overload
    def get(
        self, msg_type: Literal["OPEN_DRONE_ID_BASIC_ID"]
    ) -> mavlink.MAVLink_open_drone_id_basic_id_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["HEARTBEAT"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_heartbeat_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["PARAM_VALUE"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_param_value_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["EXTENDED_SYS_STATE"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_extended_sys_state_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["LOCAL_POSITION_NED"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_local_position_ned_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["EKF_STATUS_REPORT"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_ekf_status_report_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["GPS_RAW_INT"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_gps_raw_int_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["SYS_STATUS"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_sys_status_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["COMMAND_ACK"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_command_ack_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["MISSION_ACK"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_mission_ack_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["STATUSTEXT"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_statustext_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["MISSION_REQUEST"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_mission_request_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["MISSION_REQUEST_INT"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_mission_request_int_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["MISSION_ITEM_REACHED"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_mission_item_reached_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["MISSION_ITEM"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_mission_item_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["MISSION_COUNT"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_mission_count_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["MISSION_CURRENT"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_mission_current_message | None: ...

    @overload
    def wait_for(
        self,
        msg_type: Literal["GLOBAL_POSITION_INT"],
        timeout: float | None = None,
    ) -> mavlink.MAVLink_global_position_int_message | None: ...

    @overload
    def wait_for(
        self, msg_type: Literal["OPEN_DRONE_ID_BASIC_ID"]
    ) -> mavlink.MAVLink_open_drone_id_basic_id_message | None: ...
