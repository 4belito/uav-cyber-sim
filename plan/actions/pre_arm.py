"""
Pre-arm safety checks for UAV operation.

This module defines individual checks and a combined pre-arm `Action` to ensure
the UAV is in a safe and ready state before arming. It verifies the following:

- The UAV is disarmed
- EKF system is properly initialized
- GPS fix is sufficient (3D or better)
- Battery level is acceptable
- Required sensors are healthy

The main entry point is `make_pre_arm()`, which returns an `Action` composed of
these checks in sequence.
"""

import logging

from helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from helpers.connections.mavlink.enums import EkfStatus, ModeFlag, MsgID, SensorFlag
from helpers.connections.mavlink.streams import ask_msg, stop_msg
from plan.core import Action, ActionNames, Step, StepFailed


def make_pre_arm(
    delay: float = 0,
    ekf_flags: tuple[EkfStatus, ...] = (
        EkfStatus.ATTITUDE,
        EkfStatus.VELOCITY_HORIZ,
        EkfStatus.POS_VERT_ABS,
        EkfStatus.POS_HORIZ_ABS,
    ),
    sensor_flags: tuple[SensorFlag, ...] = (
        SensorFlag.SENSOR_3D_GYRO,
        SensorFlag.SENSOR_3D_ACCEL,
        SensorFlag.SENSOR_3D_MAG,
        SensorFlag.SENSOR_ABSOLUTE_PRESSURE,
        SensorFlag.SENSOR_GPS,
    ),
) -> Action[Step]:
    """Build a pre-arm Action that validates safety and system readiness checks."""
    pre_arm = Action[Step](name=ActionNames.PREARM, emoji="🔧")

    class CheckDisarmed(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """No execution needed; just checking."""
            pass

        def check_fn(self, conn: MAVConnection) -> bool:
            return check_disarmed(conn)

    disarm = CheckDisarmed(name="Check disarmed")

    class EFKStatus(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """No execution needed; just checking."""
            ask_msg(conn, MsgID.EKF_STATUS_REPORT)

        def check_fn(self, conn: MAVConnection) -> bool:
            return check_ekf_status(conn, ekf_flags)

    ekf_status = EFKStatus(name="Check EKF status")

    class GPSStatus(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """No execution needed; just checking."""
            ask_msg(conn, MsgID.GPS_RAW_INT)

        def check_fn(self, conn: MAVConnection) -> bool:
            return check_gps_status(conn)

    gps = GPSStatus(name="Check GPS")

    class CheckSystem(Step):
        def exec_fn(self, conn: MAVConnection) -> None:
            """Request SYS_STATUS message to check battery and sensors."""
            ask_msg(conn, MsgID.SYS_STATUS)

        def check_fn(self, conn: MAVConnection) -> bool:
            return check_sys_status(conn, sensor_flags)

    system = CheckSystem(name="Check system status")
    if delay:
        pre_arm.add(Step.make_wait(t=delay))
    for step in [disarm, ekf_status, gps, system]:
        pre_arm.add(step)
    return pre_arm


# === CHECK FUNCTIONS ===
def check_disarmed(conn: MAVConnection) -> bool:
    """Fail if the UAV is currently armed."""
    msg = conn.recv_match(type="HEARTBEAT")
    if not msg:
        return False
    if msg.base_mode & ModeFlag.SAFETY_ARMED:
        raise StepFailed("UAV is already armed")
    return True


def check_ekf_status(
    conn: MAVConnection,
    required_flags: tuple[EkfStatus, ...],
) -> bool:
    """Check whether all required EKF flags are set."""
    msg = conn.recv_match(type="EKF_STATUS_REPORT")
    if not msg:
        return False
    missing = [flag.name for flag in required_flags if not msg.flags & flag]
    if missing:
        logging.debug(
            f"🛰️ Vehicle {conn.target_system}: Waiting for EKF to be ready... "
            f"Pending: {', '.join(missing)}"
        )
        return False
    stop_msg(conn, msg_id=MsgID.EKF_STATUS_REPORT)
    return True


def check_gps_status(conn: MAVConnection) -> bool:
    """Fail if GPS fix is not 3D (fix_type < 3)."""
    msg = conn.recv_match(type="GPS_RAW_INT")
    if not msg:
        return False
    if msg.fix_type < 3:
        logging.warning(
            f"📡 Vehicle {conn.target_system}: GPS fix too weak — "
            f"fix_type = {msg.fix_type} (need at least 3 for 3D fix)"
        )
        return False
        # raise StepFailed(f"GPS fix too weak (fix_type = {msg.fix_type})")
    # stop_msg(conn, msg_id=MsgID.GPS_RAW_INT)
    return True


def check_sys_status(
    conn: MAVConnection, required_sensors: tuple[SensorFlag, ...]
) -> bool:
    """Fail if battery is low or any required sensors are unhealthy."""
    msg = conn.recv_match(type="SYS_STATUS")
    if not msg:
        return False
    if msg.battery_remaining < 20:
        raise StepFailed(
            (
                f"🔋 Vehicle {conn.target_system}: Battery too low "
                f"({msg.battery_remaining}%)"
            )
        )
    healthy = msg.onboard_control_sensors_health
    enabled = msg.onboard_control_sensors_enabled
    missing = [
        req_sensor.name
        for req_sensor in required_sensors
        if not healthy & enabled & req_sensor
    ]

    if missing:
        raise StepFailed(
            f"⚠️ Vehicle {conn.target_system}: Missing or unhealthy sensors: "
            f"{', '.join(missing)}"
        )
    stop_msg(conn, msg_id=MsgID.SYS_STATUS)
    return True
