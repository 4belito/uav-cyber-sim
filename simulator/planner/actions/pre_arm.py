"""
Pre-arm safety checks for vehicle operation.

This module defines individual checks and a combined pre-arm `Action` to ensure
the vehicle is in a safe and ready state before arming. It verifies the following:

- The vehicle is disarmed
- EKF system is properly initialized
- GPS fix is sufficient (3D or better)
- Battery level is acceptable
- Required sensors are healthy
- (plane only) All ArduPilot internal arming checks pass (gyro consistency etc.)

The main entry point is `make_pre_arm()`, which returns an `Action` composed of
these checks in sequence.
"""

import logging
import time
from typing import Literal

from simulator.helpers.connections.mavlink.enums import (
    Cmd,
    EkfStatus,
    ModeFlag,
    MsgID,
    SensorFlag,
)
from simulator.helpers.connections.mavlink.streams import ask_msg, stop_msg
from simulator.planner.action import Action
from simulator.planner.step import Step

_RUN_PREARM_CMD = int(Cmd.RUN_PREARM_CHECKS)
_MAV_RESULT_ACCEPTED = 0


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
        return not msg.base_mode & ModeFlag.SAFETY_ARMED


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


class GyroStatus(Step):
    """Wait for all present IMU gyros to be healthy and enabled.

    ArduPlane SITL simulates multiple IMUs. The 'Gyros inconsistent' prearm
    failure occurs when they haven't converged yet. Polling SYS_STATUS for
    both SENSOR_3D_GYRO and SENSOR_3D_GYRO2 health/enabled flags catches this
    before the arm attempt, following the same pattern as GPSStatus/EKFStatus.
    """

    def exec_fn(self) -> None:
        msg = ask_msg(self.conn, MsgID.SYS_STATUS)
        self.mav_manager.send(msg)

    def check_fn(self) -> bool:
        msg = self.mav_manager.state.get("SYS_STATUS")
        if not msg:
            return False
        present = msg.onboard_control_sensors_present
        healthy = msg.onboard_control_sensors_health
        enabled = msg.onboard_control_sensors_enabled
        gyros = [SensorFlag.SENSOR_3D_GYRO]
        if present & SensorFlag.SENSOR_3D_GYRO2:
            gyros.append(SensorFlag.SENSOR_3D_GYRO2)
        not_ready = [g.name for g in gyros if not (healthy & enabled & g)]
        if not_ready:
            logging.debug(
                "🔄 Vehicle %s: Waiting for gyros... Pending: %s",
                self.sysid,
                ", ".join(not_ready),
            )
            return False
        stop = stop_msg(self.conn, msg_id=MsgID.SYS_STATUS)
        self.mav_manager.send(stop)
        return True


_PREARM_RETRY_INTERVAL = 2.0


class WaitArmReady(Step):
    """Step that polls MAV_CMD_RUN_PREARM_CHECKS until all checks pass.

    Handles vehicle-specific checks not exposed via SYS_STATUS, such as
    ArduPlane's gyro-consistency requirement across multiple IMUs.
    """

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self._last_check_time: float = 0.0

    def _send_check(self) -> None:
        # Clear stale STATUSTEXT and ACK so we only read responses to this request
        self.mav_manager.state.messages.pop("STATUSTEXT", None)
        self.mav_manager.state.messages.pop("COMMAND_ACK", None)
        msg = self.conn.mav.command_long_encode(
            self.conn.target_system,
            self.conn.target_component,
            Cmd.RUN_PREARM_CHECKS,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.mav_manager.send(msg)
        self._last_check_time = time.monotonic()

    def exec_fn(self) -> None:
        """Send the pre-arm check command."""
        self._send_check()

    def check_fn(self) -> bool:
        """Return True once all ArduPilot arming checks report ready.

        MAV_CMD_RUN_PREARM_CHECKS always returns MAV_RESULT_ACCEPTED (result=0)
        to indicate the command was processed, NOT that all checks passed.
        Actual failures are broadcast as STATUSTEXT messages starting with
        'PreArm:'. We check STATUSTEXT after receiving the ACK.
        """
        ack = self.mav_manager.state.get("COMMAND_ACK")
        if ack is None or ack.command != _RUN_PREARM_CMD:
            # No ACK yet — resend if the rate-limit window has elapsed
            if time.monotonic() - self._last_check_time >= _PREARM_RETRY_INTERVAL:
                self._send_check()
            return False

        self.mav_manager.state.messages.pop("COMMAND_ACK", None)

        status = self.mav_manager.state.get("STATUSTEXT")
        if status and status.text.startswith("PreArm:"):
            logging.debug(
                "🔄 Vehicle %s: %s — retrying in %.0fs...",
                self.sysid,
                status.text,
                _PREARM_RETRY_INTERVAL,
            )
            self._last_check_time = time.monotonic()
            return False

        return True


class WaitStartupStable(Step):
    """
    Wait until observable pre-arm conditions have remained stable
    for a short local-time window.
    """

    def __init__(self, name: str, stable_seconds: float = 3.0) -> None:
        super().__init__(name)
        self.stable_seconds = stable_seconds
        self._stable_since: float | None = None

    def exec_fn(self) -> None:
        self._stable_since = None

    def check_fn(self) -> bool:
        now = time.monotonic()

        gps = self.mav_manager.state.get("GPS_RAW_INT")
        ekf = self.mav_manager.state.get("EKF_STATUS_REPORT")
        sys_status = self.mav_manager.state.get("SYS_STATUS")

        if not gps or not ekf or not sys_status:
            self._stable_since = None
            return False

        if gps.fix_type < 3:
            self._stable_since = None
            return False

        required_ekf_flags = (
            EkfStatus.ATTITUDE,
            EkfStatus.VELOCITY_HORIZ,
            EkfStatus.POS_VERT_ABS,
            EkfStatus.POS_HORIZ_ABS,
        )

        if any(not (ekf.flags & flag) for flag in required_ekf_flags):
            self._stable_since = None
            return False

        required_sensors = (
            SensorFlag.SENSOR_3D_GYRO,
            SensorFlag.SENSOR_3D_ACCEL,
            SensorFlag.SENSOR_3D_MAG,
            SensorFlag.SENSOR_ABSOLUTE_PRESSURE,
            SensorFlag.SENSOR_GPS,
        )

        healthy = sys_status.onboard_control_sensors_health
        enabled = sys_status.onboard_control_sensors_enabled

        if any(not (healthy & enabled & sensor) for sensor in required_sensors):
            self._stable_since = None
            return False

        if self._stable_since is None:
            self._stable_since = now
            logging.debug(
                "🟢 Vehicle %s: pre-arm signals stable; waiting %.1fs...",
                self.sysid,
                self.stable_seconds,
            )
            return False

        return now - self._stable_since >= self.stable_seconds


def make_pre_arm(firmware: Literal["ArduCopter", "ArduPlane"]) -> Action[Step]:
    """Build a pre-arm Action that validates safety and system readiness checks."""
    name = Action.Names.PREARM
    pre_arm = Action[Step](name=name, emoji=name.emoji)

    steps: list[Step] = [
        CheckDisarmed(name="Check disarmed"),
        EKFStatus(name="Check EKF status"),
        GPSStatus(name="Check GPS"),
        CheckSystem(name="Check system status"),
    ]
    if firmware == "ArduPlane":
        steps.append(WaitStartupStable(name="Wait startup stable", stable_seconds=3.0))
    for step in steps:
        pre_arm.add(step)
    return pre_arm
