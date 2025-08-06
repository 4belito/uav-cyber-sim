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

from functools import partial

from mavlink.customtypes.connection import MAVConnection
from mavlink.enums import EkfStatus, ModeFlag, MsgID, SensorFlag
from mavlink.util import ask_msg, stop_msg
from plan.core import Action, ActionNames, Step, StepFailed

# def noop_exec(conn: MAVConnection, verbose: int) -> None:
#     """No execution."""
#     pass


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
    # Steps
    disarm = Step(
        "Check disarmed", exec_fn=Step.noop_exec, check_fn=check_disarmed, onair=False
    )
    ekf_status = Step(
        "Check EKF status",
        check_fn=partial(check_ekf_status, required_flags=ekf_flags),
        exec_fn=partial(ask_msg, msg_id=MsgID.EKF_STATUS_REPORT),
        onair=False,
    )
    gps = Step(
        "Check GPS",
        check_fn=check_gps_status,
        exec_fn=partial(ask_msg, msg_id=MsgID.GPS_RAW_INT),
        onair=False,
    )
    system = Step(
        "Check system",
        check_fn=partial(check_sys_status, required_sensors=sensor_flags),
        exec_fn=partial(ask_msg, msg_id=MsgID.SYS_STATUS),
        onair=False,
    )
    if delay:
        pre_arm.add(Step.make_wait(t=delay))
    for step in [disarm, ekf_status, gps, system]:
        pre_arm.add(step)
    return pre_arm


# === CHECK FUNCTIONS ===
def check_disarmed(conn: MAVConnection, _verbose: int) -> tuple[bool, None]:
    """Fail if the UAV is currently armed."""
    msg = conn.recv_match(type="HEARTBEAT")
    if not msg:
        return False, None
    if msg.base_mode & ModeFlag.SAFETY_ARMED:
        raise StepFailed("UAV is already armed")
    return True, None


def check_ekf_status(
    conn: MAVConnection,
    verbose: int,
    required_flags: tuple[EkfStatus, ...],
) -> tuple[bool, None]:
    """Check whether all required EKF flags are set."""
    msg = conn.recv_match(type="EKF_STATUS_REPORT")
    if not msg:
        return False, None
    missing = [flag.name for flag in required_flags if not msg.flags & flag]
    if missing:
        if verbose > 2:
            print(
                f"Vehicle {conn.target_system}: ⌛ Waiting for EKF to be ready... "
                f"Pending: {', '.join(missing)}"
            )
        return False, None
    stop_msg(conn, msg_id=MsgID.EKF_STATUS_REPORT)
    return True, None


def check_gps_status(conn: MAVConnection, verbose: int) -> tuple[bool, None]:
    """Fail if GPS fix is not 3D (fix_type < 3)."""
    msg = conn.recv_match(type="GPS_RAW_INT")
    if not msg:
        return False, None
    if msg.fix_type < 3:
        if verbose:
            print(
                f"Vehicle {conn.target_system}: 📡 GPS fix too weak —"
                f" fix_type = {msg.fix_type} (need at least 3 for 3D fix)"
            )
            return False, None
        # raise StepFailed(f"GPS fix too weak (fix_type = {msg.fix_type})")
    # stop_msg(conn, msg_id=MsgID.GPS_RAW_INT)
    return True, None


def check_sys_status(
    conn: MAVConnection, verbose: int, required_sensors: tuple[SensorFlag, ...]
) -> tuple[bool, None]:
    """Fail if battery is low or any required sensors are unhealthy."""
    msg = conn.recv_match(type="SYS_STATUS")
    if not msg:
        return False, None
    if msg.battery_remaining < 20:
        raise StepFailed(
            f"Vehicle {conn.target_system}: Battery too low ({msg.battery_remaining}%)"
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
            f"Vehicle {conn.target_system}: Missing or unhealthy sensors: "
            f"{', '.join(missing)}"
        )
    stop_msg(conn, msg_id=MsgID.SYS_STATUS)
    return True, None
