from typing import Protocol

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
from pymavlink.dialects.v20.common import MAVLink_message


class MAVCommand:
    ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
    REQUEST_MESSAGE = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
    LOITER_UNLIMITED = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
    ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


class MAVConnection(Protocol):
    target_system: int
    target_component: int
    mav: mavlink.MAVLink

    # pylint: disable=redefined-builtin
    def recv_match(self, type: str) -> MAVLink_message | None: ...
