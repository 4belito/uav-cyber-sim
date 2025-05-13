from enum import Enum, IntEnum
from typing import Literal, Optional, Protocol, overload

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink


class MAVCommand(IntEnum):
    ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
    REQUEST_MESSAGE = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
    LOITER_UNLIMITED = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
    ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    ARM_DISARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    REQ_MSG = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
    EXT_STATE = mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE
    ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
    LOCAL_COORD = mavutil.mavlink.MAV_FRAME_LOCAL_NED
    LOCAL_POSITION_NED_ID = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED
    TAKEOFF_STATE = mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF


class RequiredSensors(IntEnum):
    """Required sensor availability flags."""

    GYRO_3D = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
    ACCEL_3D = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL
    MAG_3D = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
    ABS_PRESSURE = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    GPS = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS


class EKFFlags(IntEnum):
    """Required EKF flags for state estimation."""

    ATTITUDE = mavutil.mavlink.EKF_ATTITUDE
    VELOCITY_HORIZ = mavutil.mavlink.EKF_VELOCITY_HORIZ
    POS_HORIZ_ABS = mavutil.mavlink.EKF_POS_HORIZ_ABS
    POS_VERT_ABS = mavutil.mavlink.EKF_POS_VERT_ABS


class FlightMode(IntEnum):
    """
    Enum representing supported UAV flight modes with integer values.
    """

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

    @classmethod
    def get_name(cls, value: int) -> str:
        """Returns the name of a flight mode given its integer value."""
        for key, val in cls.__members__.items():
            if val == value:
                return key
        raise ValueError(f"No mode name for value {value}")

    @classmethod
    def get_value(cls, name: str) -> int:
        """Returns the integer value of a flight mode by name."""
        try:
            return getattr(cls, name.upper())
        except AttributeError as exc:
            raise ValueError(f"No mode value for name '{name}'") from exc


class ParamType(IntEnum):
    INT8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
    INT16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16
    INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32


class ParamName(bytes, Enum):
    NAV_SPEED = b"WPNAV_SPEED"


# For annotation
class ParamValueMessage(Protocol):
    param_id: bytes
    param_value: float


class HeartbeatMessage(Protocol):
    base_mode: int
    custom_mode: int


class SysStateMessage(Protocol):
    landed_state: int


class LocalPositionMessage(Protocol):
    x: float
    y: float
    z: float


class EKFStatusReportMessage(Protocol):
    flags: int


class GPSRawIntMessage(Protocol):
    fix_type: int


class SysStatusMessage(Protocol):
    battery_remaining: int
    onboard_control_sensors_health: int


class MAVConnection(Protocol):
    target_system: int
    target_component: int
    mav: mavlink.MAVLink

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

    def set_mode(self, mode: int) -> None: ...
