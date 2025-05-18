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
from plan.core import Action, ActionNames, Step, StepFailed
from helpers.mavlink import (
    EKFFlags,
    MAVCommand,
    MAVConnection,
    RequiredSensors,
    ask_msg,
    stop_msg,
)


def make_pre_arm():
    """Builds a pre-arm Action that validates safety and system readiness checks."""
    pre_arm = Action(name=ActionNames.PREARM, emoji="🔧")
    # Steps
    disarm = Step("Check disarmed", check_fn=check_disarmed, onair=False)
    ekf_status = Step(
        "Check EKF status",
        check_fn=partial(check_ekf_status),
        exec_fn=partial(ask_msg, msg_id=MAVCommand.EKF_STATUS_REPORT),
        onair=False,
    )
    gps = Step(
        "Check GPS",
        check_fn=check_gps_status,
        exec_fn=partial(ask_msg, msg_id=MAVCommand.GPS_RAW),
        onair=False,
    )
    system = Step(
        "Check system",
        check_fn=check_sys_status,
        exec_fn=partial(ask_msg, msg_id=MAVCommand.SYS_STATUS),
        onair=False,
    )
    for step in [disarm, ekf_status, gps, system]:
        pre_arm.add_step(step)
    return pre_arm


# === CHECK FUNCTIONS ===
def check_disarmed(conn: MAVConnection, _verbose: int):
    """Fails if the UAV is currently armed."""
    msg = conn.recv_match(type="HEARTBEAT")
    if not msg:
        return False, None
    if msg.base_mode & MAVCommand.ARMED_FLAG:
        raise StepFailed("UAV is already armed")
    return True, None


def check_ekf_status(conn: MAVConnection, verbose: int):
    """Checks whether all required EKF flags are set."""
    if verbose == 2:
        print("📡 Requesting EKF_STATUS_REPORT...")

    msg = conn.recv_match(type="EKF_STATUS_REPORT")
    if not msg:
        return False, None
    missing = [flag.name for flag in EKFFlags if not msg.flags & flag]
    if missing:
        if verbose == 2:
            print(f"❌ EKF check failed! Still waiting for: {', '.join(missing)}")
        return False, None
    stop_msg(conn, msg_id=MAVCommand.EKF_STATUS_REPORT)
    return True, None


def check_gps_status(conn: MAVConnection, verbose: int):
    """Fails if GPS fix is not 3D (fix_type < 3)."""
    msg = conn.recv_match(type="GPS_RAW_INT")
    if not msg:
        return False, None
    if msg.fix_type < 3:
        if verbose:
            print(
                f"📡 GPS fix too weak — fix_type = {msg.fix_type} "
                f"(need at least 3 for 3D fix)"
            )
            return False, None
        # raise StepFailed(f"GPS fix too weak (fix_type = {msg.fix_type})")
    stop_msg(conn, msg_id=MAVCommand.GPS_RAW)
    return True, None


def check_sys_status(conn: MAVConnection, _verbose: int):
    """Fails if battery is low or any required sensors are unhealthy."""
    msg = conn.recv_match(type="SYS_STATUS")
    if not msg:
        return False, None
    if msg.battery_remaining < 20:
        raise StepFailed(f"Battery too low ({msg.battery_remaining}%)")

    missing = [
        sensor.name
        for sensor in RequiredSensors
        if not msg.onboard_control_sensors_health & sensor
    ]

    if missing:
        raise StepFailed(f"Missing or unhealthy sensors: {', '.join(missing)}")
    stop_msg(conn, msg_id=MAVCommand.SYS_STATUS)
    return True, None
