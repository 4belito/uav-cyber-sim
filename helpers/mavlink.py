"""
MAVLink command and message type definitions.

Provides enums for MAVLink commands, flight modes, parameter types, and required sensor
flags. Also defines protocol interfaces for typed access to MAVLink messages and
connections.
"""

from enum import Enum, IntEnum
from typing import Literal, Optional, Protocol, overload

from pymavlink import mavutil  # type: ignore
from pymavlink.dialects.v20 import common as mavlink  # type: ignore

MissionResult = IntEnum(  # type: ignore
    "MAV_MISSION_RESULT",
    {v.name: k for k, v in mavutil.mavlink.enums["MAV_MISSION_RESULT"].items()},
)

NavCommand = IntEnum(  # type: ignore
    "MAV_CMD_NAV",
    {
        v.name: k
        for k, v in mavutil.mavlink.enums["MAV_CMD"].items()
        if v.name.startswith("MAV_CMD_NAV_")
    },
)


# TODO improve this
class MavCmd(IntEnum):
    """Common MAVLink commands and related constants used in UAV control."""

    # Navegation
    TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
    LOITER_UNLIMITED = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM

    SET_MESSAGE_INTERVAL = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
    ARM_DISARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    REQ_MSG = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
    SET_MODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE

    EXT_STATE = mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
    ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    LOCAL_COORD = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    LOCAL_POSITION_NED_ID = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
    TAKEOFF_STATE = mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF
    SYS_STATUS = mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS
    ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    CUSTOM_MODE_ENABLE = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    GPS_RAW = mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT
    EKF_STATUS_REPORT = mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT
    START_MISSION = mavutil.mavlink.MAV_CMD_MISSION_START
    TYPE_GCS = mavutil.mavlink.MAV_TYPE_GCS
    AUTOPILOT_INVALID = mavutil.mavlink.MAV_AUTOPILOT_INVALID


class CustomCmd(IntEnum):
    """official MAV_CMD values generally range from 0 to ~2999."""

    PLAN_DONE = 3000  # Custom command to mark end of plan


class RequiredSensors(IntEnum):
    """Sensor availability flags used in SYS_STATUS messages."""

    GYRO_3D = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
    ACCEL_3D = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL
    MAG_3D = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
    ABS_PRESSURE = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    GPS = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS


class EKFFlags(IntEnum):
    """EKF status flags indicating available state estimates."""

    ATTITUDE = mavutil.mavlink.EKF_ATTITUDE
    VELOCITY_HORIZ = mavutil.mavlink.EKF_VELOCITY_HORIZ
    POS_VERT_ABS = mavutil.mavlink.EKF_POS_VERT_ABS
    # comment this in deboug mode for saving a fcouple of seconds during arming
    POS_HORIZ_ABS = mavutil.mavlink.EKF_POS_HORIZ_ABS


class FlightMode(IntEnum):
    """Supported ArduPilot flight modes with helpers for name↔value conversion."""

    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTO_TUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    AVOID_ADSB = 19
    GUIDED_NO_GPS = 20
    SMART_RTL = 21
    FLOWHOLD = 22
    FOLLOW = 23
    ZIGZAG = 24
    SYSTEM_ID = 25


# class MissionAckType(IntEnum):
#     """Acknowledgment types for MAVLink mission operations."""

#     MAV_MISSION_ACCEPTED = 0  # mission accepted OK
#     MAV_MISSION_ERROR = 1  # Generic error / not accepting mission commands
#     MAV_MISSION_UNSUPPORTED_FRAME = 2  # Coordinate frame is not supported
#     MAV_MISSION_UNSUPPORTED = 3  # Command is not supported
#     MAV_MISSION_NO_SPACE = 4  # Mission items exceed storage space
#     MAV_MISSION_INVALID = 5  # One of the parameters has an invalid value
#     MAV_MISSION_INVALID_PARAM1 = 6
#     MAV_MISSION_INVALID_PARAM2 = 7
#     MAV_MISSION_INVALID_PARAM3 = 8
#     MAV_MISSION_INVALID_PARAM4 = 9
#     MAV_MISSION_INVALID_PARAM5_X = 10
#     MAV_MISSION_INVALID_PARAM6_Y = 11
#     MAV_MISSION_INVALID_PARAM7 = 12
#     MAV_MISSION_INVALID_SEQUENCE = 13  # Mission item received out of sequence
#     MAV_MISSION_DENIED = 14  # Not accepting any mission commands from this partner
#     MAV_MISSION_OPERATION_CANCELLED = 15  # Current mission operation was cancelled


class ParamType(IntEnum):
    """Parameter types used in PARAM_VALUE messages."""

    INT8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
    INT16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16
    INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32


class ParamName(bytes, Enum):
    """Predefined parameter names used in configuration (e.g., NAV_SPEED)."""

    NAV_SPEED = b"WPNAV_SPEED"


# pylint: disable=too-few-public-methods
class ParamValueMessage(Protocol):
    """Typed protocol for the MAVLink PARAM_VALUE message."""

    param_id: bytes
    param_value: float


class HeartbeatMessage(Protocol):
    """Protocol for the MAVLink HEARTBEAT message."""

    base_mode: int
    custom_mode: int


class SysStateMessage(Protocol):
    """Protocol for the MAVLink EXTENDED_SYS_STATE message."""

    landed_state: int


class LocalPositionMessage(Protocol):
    """Protocol for the MAVLink LOCAL_POSITION_NED message."""

    x: float
    y: float
    z: float


class EKFStatusReportMessage(Protocol):
    """Protocol for the MAVLink EKF_STATUS_REPORT message."""

    flags: int


class GPSRawIntMessage(Protocol):
    """Protocol for the MAVLink GPS_RAW_INT message."""

    fix_type: int


class SysStatusMessage(Protocol):
    """Protocol for the MAVLink SYS_STATUS message."""

    battery_remaining: int
    onboard_control_sensors_health: int


class ACKMessage(Protocol):
    """Protocol for the MAVLink SYS_STATUS message."""

    command: int


class TextMessage(Protocol):
    """Protocol for the MAVLink SYS_STATUS message."""

    text: str


class MAVLinkMessage(Protocol):
    """Generic MAVLink message interface with serialization helpers."""

    def get_type(self) -> str:
        """Return the type name of the MAVLink message."""
        raise NotImplementedError("This method must be implemented by subclasses.")

    def get_msgbuf(self) -> bytes:
        """Return the raw MAVLink message as a bytes buffer."""
        raise NotImplementedError("This method must be implemented by subclasses.")


class MissionItemIntMessage(Protocol):
    """Protocol for the MAVLink MISSION_ITEM_INT message."""

    target_system: int
    target_component: int
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float
    mission_type: int


class MissionRequestMessage(Protocol):
    """Protocol for MAVLink MISSION_REQUEST or MISSION_REQUEST_INT message."""

    target_system: int
    target_component: int
    seq: int


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
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[MAVLinkMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["HEARTBEAT"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
    ) -> Optional[HeartbeatMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["PARAM_VALUE"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
    ) -> Optional[ParamValueMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["EXTENDED_SYS_STATE"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[SysStateMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["LOCAL_POSITION_NED"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[LocalPositionMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["EKF_STATUS_REPORT"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[EKFStatusReportMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["GPS_RAW_INT"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[GPSRawIntMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["SYS_STATUS"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[SysStatusMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["COMMAND_ACK"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[ACKMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["STATUSTEXT"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[TextMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["MISSION_REQUEST"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[MissionRequestMessage]: ...

    @overload
    def recv_match(
        self,
        type: Literal["MISSION_REQUEST_INT"],  # pylint: disable=redefined-builtin
        timeout: Optional[float] = ...,
        blocking: Optional[bool] = ...,
    ) -> Optional[MissionRequestMessage]: ...

    def recv_msg(self) -> Optional[MAVLinkMessage]:
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

    def send(self, mavmsg: MissionItemIntMessage) -> None:
        """Send the mission item."""


def ask_msg(conn: MAVConnection, msg_id: int, interval: int = 1_000_000) -> None:
    """Request periodic sending of a MAVLink message (1 Hz)."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        MavCmd.SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval,  # microseconds
        0,
        0,
        0,
        0,
        0,
    )


def stop_msg(conn: MAVConnection, msg_id: int) -> None:
    """Stop sending a specific MAVLink message."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        MavCmd.SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        -1,  # Stop
        0,
        0,
        0,
        0,
        0,
    )
