"""
Pre-arm safety checks for vehicle operation.

This version is better for ArduPlane because it also watches STATUSTEXT for
explicit pre-arm failures such as:
- "Arm: Gyros inconsistent"
- "PreArm: ..."

It keeps the generic checks from the original module, but adds a Plane-friendly
failure detector instead of relying only on SYS_STATUS sensor flags.
"""

from __future__ import annotations

import logging
from typing import Literal

from simulator.helpers.connections.mavlink.enums import (
    EkfStatus,
    ModeFlag,
    MsgID,
    SensorFlag,
)
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.planner.action import Action
from simulator.planner.step import Step

VehicleType = Literal["plane", "copter"]


class CheckDisarmed(Step):
    """Step to verify that the vehicle is disarmed before arming."""

    def exec_fn(self) -> None:
        """Request HEARTBEAT."""
        msg = ask_msg(self.conn, MsgID.HEARTBEAT)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Return True only when the vehicle is disarmed."""
        msg = self.mav_manager.state.get("HEARTBEAT")
        if not msg:
            return False
        return (msg.base_mode & ModeFlag.SAFETY_ARMED) == 0


class EKFStatus(Step):
    """Step to verify that the EKF system is properly initialized."""

    def __init__(
        self,
        name: str,
        required_ekf_flags: tuple[EkfStatus, ...],
    ):
        super().__init__(name)
        self.required_ekf_flags = required_ekf_flags

    def exec_fn(self) -> None:
        """Request EKF status."""
        msg = ask_msg(self.conn, MsgID.EKF_STATUS_REPORT)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Check whether all required EKF flags are set."""
        msg = self.mav_manager.state.get("EKF_STATUS_REPORT")
        if not msg:
            return False

        missing = [
            flag.name for flag in self.required_ekf_flags if not (msg.flags & flag)
        ]
        if missing:
            logging.debug(
                f"🛰️ Vehicle {self.sysid}: Waiting for EKF to be ready... "
                f"Pending: {', '.join(missing)}"
            )
            return False

        msg = stop_msg(self.conn, msg_id=MsgID.EKF_STATUS_REPORT)
        self.mav_manager.send(msg)
        return True


class GPSStatus(Step):
    """Step to verify that the GPS fix is sufficient."""

    def exec_fn(self) -> None:
        """Request GPS status."""
        msg = ask_msg(self.conn, MsgID.GPS_RAW_INT)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Require at least 3D fix."""
        msg = self.mav_manager.state.get("GPS_RAW_INT")
        if not msg:
            return False

        if msg.fix_type < 3:
            logging.warning(
                f"📡 Vehicle {self.sysid}: GPS fix too weak — "
                f"fix_type = {msg.fix_type} (need at least 3 for 3D fix)"
            )
            return False

        return True


class CheckSystem(Step):
    """Generic system status check using SYS_STATUS."""

    def __init__(
        self,
        name: str,
        required_sensors: tuple[SensorFlag, ...],
    ):
        super().__init__(name)
        self.required_sensors = required_sensors

    def exec_fn(self) -> None:
        """Request SYS_STATUS."""
        msg = ask_msg(self.conn, MsgID.SYS_STATUS)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Fail if battery is low or required sensors are unhealthy."""
        msg = self.mav_manager.state.get("SYS_STATUS")
        if not msg:
            return False

        if msg.battery_remaining != -1 and msg.battery_remaining < 20:
            raise Exception(
                f"🔋 Vehicle {self.sysid}: Battery too low ({msg.battery_remaining}%)"
            )

        healthy = msg.onboard_control_sensors_health
        enabled = msg.onboard_control_sensors_enabled

        missing = [
            sensor.name
            for sensor in self.required_sensors
            if not (healthy & enabled & sensor)
        ]

        if missing:
            raise Exception(
                f"⚠️ Vehicle {self.sysid}: Missing or unhealthy sensors: "
                f"{', '.join(missing)}"
            )

        msg = stop_msg(self.conn, msg_id=MsgID.SYS_STATUS)
        self.mav_manager.send(msg)
        return True


class CheckPreArmText(Step):
    """
    Detect explicit ArduPilot pre-arm failures from STATUSTEXT.

    This is important for Plane because failures like
    'Arm: Gyros inconsistent' may not be inferable from SYS_STATUS alone.
    """

    FAIL_PATTERNS = (
        "prearm:",
        "arm:",
        "gyros inconsistent",
        "accels inconsistent",
        "ahrs not healthy",
        "ekf not ready",
        "gps",
        "compass",
        "baro",
        "airspeed",
    )

    def exec_fn(self) -> None:
        """Request STATUSTEXT stream."""
        msg = ask_msg(self.conn, MsgID.STATUSTEXT)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """
        Return False while pre-arm errors are present.

        Raise on explicit failure text so the plan stops with a meaningful cause.
        """
        msg = self.mav_manager.state.get("STATUSTEXT")
        if not msg:
            return False

        text = msg.text.strip()
        lowered = text.lower()

        if any(pattern in lowered for pattern in self.FAIL_PATTERNS):
            raise Exception(f"⚠️ Vehicle {self.sysid}: Pre-arm failure: {text}")

        # If we have any status text and none of it looks like a failure,
        # consider this check satisfied.
        try:
            msg = stop_msg(self.conn, msg_id=MsgID.STATUSTEXT)
            self.mav_manager.send(msg)
        except Exception:
            pass
        return True


def make_pre_arm(vehicle_type: VehicleType = "plane") -> Action[Step]:
    """Build a pre-arm action with checks appropriate for the vehicle type."""
    name = Action.Names.PREARM
    pre_arm = Action[Step](name=name, emoji=name.emoji)

    if vehicle_type == "plane":
        required_ekf_flags = (
            EkfStatus.ATTITUDE,
            EkfStatus.VELOCITY_HORIZ,
            EkfStatus.POS_HORIZ_ABS,
        )
        required_sensors = (
            SensorFlag.SENSOR_3D_GYRO,
            SensorFlag.SENSOR_3D_ACCEL,
            SensorFlag.SENSOR_ABSOLUTE_PRESSURE,
            SensorFlag.SENSOR_GPS,
        )
    else:
        required_ekf_flags = (
            EkfStatus.ATTITUDE,
            EkfStatus.VELOCITY_HORIZ,
            EkfStatus.POS_VERT_ABS,
            EkfStatus.POS_HORIZ_ABS,
        )
        required_sensors = (
            SensorFlag.SENSOR_3D_GYRO,
            SensorFlag.SENSOR_3D_ACCEL,
            SensorFlag.SENSOR_3D_MAG,
            SensorFlag.SENSOR_ABSOLUTE_PRESSURE,
            SensorFlag.SENSOR_GPS,
        )

    steps: list[Step] = [
        CheckDisarmed(name="Check disarmed"),
        EKFStatus(name="Check EKF status", required_ekf_flags=required_ekf_flags),
        GPSStatus(name="Check GPS"),
        CheckSystem(name="Check system status", required_sensors=required_sensors),
        CheckPreArmText(name="Check pre-arm status text"),
    ]

    for step in steps:
        pre_arm.add(step)

    return pre_arm
