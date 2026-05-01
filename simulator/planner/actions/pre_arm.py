"""
Pre-arm safety checks for vehicle operation.

This module defines individual checks and a combined pre-arm `Action` to ensure
the vehicle is in a safe and ready state before arming. It verifies the following:

- The vehicle is disarmed
- EKF system is properly initialized
- GPS fix is sufficient (3D or better)
- Battery level is acceptable
- Required sensors are healthy

The main entry point is `make_pre_arm()`, which returns an `Action` composed of
these checks in sequence.
"""

import logging

from simulator.helpers.connections.mavlink.enums import (
    EkfStatus,
    ModeFlag,
    MsgID,
    SensorFlag,
)
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.planner.action import Action
from simulator.planner.step import Step


class CheckDisarmed(Step):
    """Step to verify that the vehicle is disarmed before arming."""

    def exec_fn(self) -> None:
        """No execution needed; just checking."""
        pass

    def check_fn(self) -> bool:
        """Fail if the vehicle is currently armed."""
        msg = self.mav_manager.state.get("HEARTBEAT")
        if not msg:
            return False
        if msg.base_mode & ModeFlag.SAFETY_ARMED:
            return False
        return True


class EKFStatus(Step):
    """Step to verify that the EKF system is properly initialized."""

    def __init__(
        self,
        name: str,
        required_ekf_flags: tuple[EkfStatus, ...] = (
            EkfStatus.ATTITUDE,
            EkfStatus.VELOCITY_HORIZ,
            EkfStatus.POS_VERT_ABS,
            EkfStatus.POS_HORIZ_ABS,
        ),
    ):
        super().__init__(name)
        self.required_ekf_flags = required_ekf_flags

    def exec_fn(self) -> None:
        """No execution needed; just checking."""
        msg = ask_msg(self.conn, MsgID.EKF_STATUS_REPORT)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Check whether all required EKF flags are set."""
        msg = self.mav_manager.state.get("EKF_STATUS_REPORT")
        if not msg:
            return False
        missing = [
            flag.name for flag in self.required_ekf_flags if not msg.flags & flag
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
        """No execution needed; just checking."""
        msg = ask_msg(self.conn, MsgID.GPS_RAW_INT)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Fail if GPS fix is not 3D (fix_type < 3)."""
        msg = self.mav_manager.state.get("GPS_RAW_INT")
        if not msg:
            return False
        if msg.fix_type < 3:
            logging.warning(
                f"📡 Vehicle {self.sysid}: GPS fix too weak — "
                f"fix_type = {msg.fix_type} (need at least 3 for 3D fix)"
            )
            return False
            # raise StepFailed(f"GPS fix too weak (fix_type = {msg.fix_type})")
        # stop_msg(conn, msg_id=MsgID.GPS_RAW_INT)
        return True


class CheckSystem(Step):
    """Step to verify overall system status including battery and sensors."""

    def __init__(
        self,
        name: str,
        required_sensors: tuple[SensorFlag, ...] = (
            SensorFlag.SENSOR_3D_GYRO,
            SensorFlag.SENSOR_3D_ACCEL,
            SensorFlag.SENSOR_3D_MAG,
            SensorFlag.SENSOR_ABSOLUTE_PRESSURE,
            SensorFlag.SENSOR_GPS,
        ),
    ):
        super().__init__(name)
        self.required_sensors = required_sensors

    def exec_fn(self) -> None:
        """Request SYS_STATUS message to check battery and sensors."""
        msg = ask_msg(self.conn, MsgID.SYS_STATUS)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        """Fail if battery is low or any required sensors are unhealthy."""
        msg = self.mav_manager.state.get("SYS_STATUS")
        if not msg:
            return False
        if msg.battery_remaining < 20:
            raise Exception(
                f"🔋 Vehicle {self.sysid}: Battery too low ({msg.battery_remaining}%)"
            )
        healthy = msg.onboard_control_sensors_health
        enabled = msg.onboard_control_sensors_enabled
        missing = [
            req_sensor.name
            for req_sensor in self.required_sensors
            if not healthy & enabled & req_sensor
        ]

        if missing:
            raise Exception(
                f"⚠️ Vehicle {self.sysid}: Missing or unhealthy sensors: "
                f"{', '.join(missing)}"
            )
        msg = stop_msg(self.conn, msg_id=MsgID.SYS_STATUS)
        self.mav_manager.send(msg)
        return True


def make_pre_arm() -> Action[Step]:
    """Build a pre-arm Action that validates safety and system readiness checks."""
    name = Action.Names.PREARM
    pre_arm = Action[Step](name=name, emoji=name.emoji)

    disarm = CheckDisarmed(name="Check disarmed")
    ekf_status = EKFStatus(name="Check EKF status")
    gps = GPSStatus(name="Check GPS")
    system = CheckSystem(name="Check system status")
    for step in [disarm, ekf_status, gps, system]:
        pre_arm.add(step)
    return pre_arm
